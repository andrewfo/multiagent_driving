import asyncio
import websockets
import json

# Program Description: 
# receives a JSON message from one car and broadcasts it to every other connected car;
# can be run on main laptop that will host the websocket server

# Keep track of all connected cars
connected_cars = set()

async def handle_car_connection(websocket, path):
    # Register the new car
    connected_cars.add(websocket)
    print(f"New car connected. Total cars: {len(connected_cars)}")
    
    try:
        async for message in websocket:
            # When a car pushes its information (pose, size, velocity)
            # Broadcast it to all *other* cars
            for car in connected_cars:
                if car != websocket:
                    await car.send(message)
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        # Unregister the car if it disconnects
        connected_cars.remove(websocket)
        print(f"Car disconnected. Total cars: {len(connected_cars)}")

async def main():
    # Start the server on port 8765, listening on all network interfaces
    async with websockets.serve(handle_car_connection, "0.0.0.0", 8765):
        print("Centralized WebSocket Server running on ws://0.0.0.0:8765")
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    asyncio.run(main())