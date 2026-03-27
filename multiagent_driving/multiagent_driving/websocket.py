#!/usr/bin/env python3
"""
Centralized WebSocket server for multi-agent ROS2 driving.

Run this on one car (or a laptop on the same network). It receives JSON
state messages from each connected car and broadcasts them to all others.
It also routes command messages from a commander to individual cars.

Protocol – two message types:

  State (car -> server -> all peers):
  {
    "car_id":       "<namespace>",
    "x":            <float>,
    "y":            <float>,
    "yaw":          <float>,
    "vx":           <float>,              // linear velocity
    "status":       "navigating"|"idle",  // derived from velocity
    "current_goal": {"x": <f>, "y": <f>} | null,
    "obstacles":    [ {"x": <f>, "y": <f>}, ... ]   // optional
  }

  Command (commander -> server -> target car):
  {
    "type":       "command",
    "target_car": "<car_id>",
    "action":     "<action_name>",
    ...action-specific fields...
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

# car_id -> websocket handle (for targeted command routing)
car_sockets: dict = {}

# All currently connected websocket handles
connected: set = set()


async def handle_connection(websocket):
    """Handle a single car's (or commander's) lifecycle: register, relay, unregister."""
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
            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                continue

            # --- Command messages: route to target car only ---
            if data.get("type") == "command":
                target = data.get("target_car")
                if target and target in car_sockets:
                    try:
                        await car_sockets[target].send(raw)
                        print(f"[>] Command -> {target}: {data.get('action', '?')}")
                    except websockets.exceptions.ConnectionClosed:
                        print(f"[!] Failed to send command to {target} (disconnected)")
                else:
                    print(f"[!] Command for unknown car: {target}")
                continue

            # --- State messages: cache and broadcast to peers ---
            car_id = data.get("car_id")
            if car_id is None:
                continue

            car_sockets[car_id] = websocket
            latest_state[car_id] = raw

            # Broadcast to every *other* connected client
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
        if car_id:
            latest_state.pop(car_id, None)
            car_sockets.pop(car_id, None)
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
