#!/usr/bin/env python3
"""
Centralized WebSocket server for multi-agent ROS2 driving.

Run this on one car (or a laptop on the same network). It receives JSON
state messages from each connected car and broadcasts them to all others.

Protocol – every message is a JSON object with at least:
  {
    "car_id":    "<namespace>",
    "x":         <float>,
    "y":         <float>,
    "yaw":       <float>,
    "obstacles": [ {"x": <float>, "y": <float>}, ... ]   # optional
  }

New connections immediately receive the latest known state of every other
car so they can populate their costmap without waiting for the next update.
"""

import argparse
import asyncio
import json
import signal
import websockets

# car_id -> latest JSON state (kept as raw string for fast relay)
latest_state: dict[str, str] = {}

# All currently connected websocket handles
connected: set = set()


async def handle_connection(websocket):
    """Handle a single car's lifecycle: register, relay, unregister."""
    connected.add(websocket)
    car_id = None
    print(f"[+] New connection ({len(connected)} total)")

    # Send the newcomer a snapshot of every other car's last-known state
    for state_msg in latest_state.values():
        try:
            await websocket.send(state_msg)
        except websockets.exceptions.ConnectionClosed:
            break

    try:
        async for raw in websocket:
            # Parse just enough to extract car_id and cache the message
            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                continue

            car_id = data.get("car_id")
            if car_id is None:
                continue

            latest_state[car_id] = raw

            # Broadcast to every *other* connected car
            peers = [c for c in connected if c is not websocket]
            if peers:
                await asyncio.gather(
                    *(peer.send(raw) for peer in peers),
                    return_exceptions=True,
                )
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        connected.discard(websocket)
        if car_id and car_id in latest_state:
            del latest_state[car_id]
        label = car_id or "unknown"
        print(f"[-] {label} disconnected ({len(connected)} remaining)")


async def run_server(host: str, port: int):
    stop = asyncio.get_event_loop().create_future()

    loop = asyncio.get_event_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set_result, None)

    async with websockets.serve(handle_connection, host, port):
        print(f"WebSocket server listening on ws://{host}:{port}")
        await stop
        print("\nShutting down.")


def main():
    parser = argparse.ArgumentParser(description="Multi-agent WebSocket relay server")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8765, help="Port (default: 8765)")
    args = parser.parse_args()

    asyncio.run(run_server(args.host, args.port))


if __name__ == "__main__":
    main()
