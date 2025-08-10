import asyncio
import json
import websockets

# 板子的TCP服务器地址和端口
TCP_SERVER_IP = "192.168.0.106"
TCP_SERVER_PORT = 5000

# WebSocket服务器的地址和端口
WS_SERVER_IP = "0.0.0.0"
WS_SERVER_PORT = 8765

# 连接到WebSocket的客户端集合
clients = set()

async def broadcast_status():
    """
    维持一个到TCP服务器的持久连接，
    定期获取状态并广播给所有WebSocket客户端。
    """
    while True: # 外层循环用于处理连接断开后的自动重连
        try:
            reader, writer = await asyncio.open_connection(TCP_SERVER_IP, TCP_SERVER_PORT)
            print(f"Successfully connected to TCP server at {TCP_SERVER_IP}:{TCP_SERVER_PORT}")

            while True: # 内层循环用于在已建立的连接上持续通信
                writer.write(b"get_status\r\n")
                await writer.drain()
                # 注意：如果服务器没有立即响应，read可能会阻塞或返回空
                data = await reader.read(1024)
                
                if not data:
                    print("TCP server closed the connection. Reconnecting...")
                    break # 跳出内层循环，触发重连

                try:
                    status_str = data.decode('utf-8')
                    json_start = status_str.find('{')
                    if json_start != -1:
                        status_str = status_str[json_start:]
                        status = json.loads(status_str)
                        
                        if status and clients:
                            message = json.dumps(status)
                            # 并发地将消息发送给所有WebSocket客户端
                            await asyncio.gather(
                                *[client.send(message) for client in clients]
                            )
                except (UnicodeDecodeError, json.JSONDecodeError) as e:
                    print(f"Error decoding/parsing JSON: {e}")
                    print(f"Received raw data: {data}")
                
                await asyncio.sleep(0.1)

        except (ConnectionRefusedError, OSError) as e:
            print(f"Failed to connect to TCP server: {e}. Retrying in 5 seconds...")
            await asyncio.sleep(5)
        except Exception as e:
            print(f"An unexpected error occurred: {e}. Retrying in 5 seconds...")
            await asyncio.sleep(5)

async def register(websocket):
    """注册新的客户端连接"""
    clients.add(websocket)
    print(f"New client connected. Total clients: {len(clients)}")
    try:
        await websocket.wait_closed()
    finally:
        clients.remove(websocket)
        print(f"Client disconnected. Total clients: {len(clients)}")

async def main():
    """主函数，启动WebSocket服务器和状态广播任务"""
    server = await websockets.serve(register, WS_SERVER_IP, WS_SERVER_PORT)
    print(f"WebSocket server started at ws://{WS_SERVER_IP}:{WS_SERVER_PORT}")
    
    # 启动状态广播任务
    asyncio.create_task(broadcast_status())
    
    await server.wait_closed()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Server stopped.")
