import socket
import json
import base64
import hashlib
import struct
import time
import threading

class WebSocketManager:
    def __init__(self, ip: str, port: int, local_ip: str):
        self.ip = ip
        self.port = port
        self.local_ip = local_ip
        self.sock = None

    def connect(self):
        if self.sock is None:
            try:
                # Create a simple socket connection to ROSBridge
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect((self.ip, self.port))
                
                # Perform WebSocket handshake
                self._websocket_handshake()
                print("[WebSocket] Connected")
            except Exception as e:
                print(f"[WebSocket] Connection error: {e}")
                self.sock = None

    def _websocket_handshake(self):
        # Simple WebSocket handshake
        key = base64.b64encode(b"ros-mcp-server-key").decode()
        handshake = (
            f"GET / HTTP/1.1\r\n"
            f"Host: {self.ip}:{self.port}\r\n"
            f"Upgrade: websocket\r\n"
            f"Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            f"Sec-WebSocket-Version: 13\r\n"
            f"\r\n"
        )
        self.sock.send(handshake.encode())
        
        # Read handshake response
        response = self.sock.recv(1024).decode()
        if "101 Switching Protocols" not in response:
            raise Exception(f"WebSocket handshake failed: {response}")

    def send(self, message: dict, reconnect: bool = True):
        if reconnect:
            self.connect()
        if self.sock:
            try:
                # Ensure message is JSON serializable
                json_msg = json.dumps(message)
                self._send_frame(json_msg)
            except TypeError as e:
                print(f"[WebSocket] JSON serialization error: {e}")
                self.close()
            except Exception as e:
                print(f"[WebSocket] Send error: {e}")
                self.close()

    def _send_frame(self, data: str):
        # Send WebSocket frame
        data_bytes = data.encode('utf-8')
        frame = bytearray()
        
        # FIN + opcode (text frame)
        frame.append(0x81)
        
        # Payload length
        length = len(data_bytes)
        if length < 126:
            frame.append(length | 0x80)  # Masked
        elif length < 65536:
            frame.append(126 | 0x80)
            frame.extend(struct.pack('>H', length))
        else:
            frame.append(127 | 0x80)
            frame.extend(struct.pack('>Q', length))
        
        # Masking key (simple key for ROSBridge)
        mask = b'\x00\x00\x00\x00'
        frame.extend(mask)
        
        # Masked payload
        frame.extend(data_bytes)
        
        self.sock.send(frame)

    def _receive_frame(self) -> str:
        # Receive WebSocket frame
        header = self.sock.recv(2)
        if len(header) < 2:
            return ""
        
        # Parse frame header
        fin = (header[0] & 0x80) != 0
        opcode = header[0] & 0x0f
        masked = (header[1] & 0x80) != 0
        length = header[1] & 0x7f
        
        # Get payload length
        if length == 126:
            length_data = self.sock.recv(2)
            length = struct.unpack('>H', length_data)[0]
        elif length == 127:
            length_data = self.sock.recv(8)
            length = struct.unpack('>Q', length_data)[0]
        
        # Get mask if present
        if masked:
            mask = self.sock.recv(4)
        
        # Get payload
        payload = self.sock.recv(length)
        
        # Unmask if needed
        if masked:
            payload = bytes(payload[i] ^ mask[i % 4] for i in range(len(payload)))
        
        return payload.decode('utf-8')

    def subscribe_once(self, topic: str, timeout: float = 5.0) -> dict:
        """
        订阅话题并获取一条消息
        
        Args:
            topic: 话题名称
            timeout: 超时时间（秒）
            
        Returns:
            消息字典，如果失败返回None
        """
        self.connect()
        if not self.sock:
            return None
            
        try:
            # 生成唯一的订阅ID
            subscribe_id = f"subscribe_{topic.replace('/', '_')}_{int(time.time() * 1000)}"
            
            # 发送订阅请求
            subscribe_msg = {
                "op": "subscribe",
                "topic": topic,
                "id": subscribe_id
            }
            self.send(subscribe_msg, reconnect=False)
            
            # 设置socket超时
            self.sock.settimeout(timeout)
            
            start_time = time.time()
            while time.time() - start_time < timeout:
                try:
                    response = self._receive_frame()
                    if response:
                        data = json.loads(response)
                        
                        # 检查是否是我们订阅的话题的消息
                        if (data.get("op") == "publish" and 
                            data.get("topic") == topic):
                            
                            # 取消订阅
                            unsubscribe_msg = {
                                "op": "unsubscribe",
                                "topic": topic,
                                "id": subscribe_id
                            }
                            self.send(unsubscribe_msg, reconnect=False)
                            
                            return data
                            
                except socket.timeout:
                    continue
                except json.JSONDecodeError:
                    continue
                    
            # 超时后取消订阅
            try:
                unsubscribe_msg = {
                    "op": "unsubscribe", 
                    "topic": topic,
                    "id": subscribe_id
                }
                self.send(unsubscribe_msg, reconnect=False)
            except:
                pass
                
            return None
            
        except Exception as e:
            print(f"[WebSocket] Subscribe once error: {e}")
            return None
        finally:
            # 重置socket超时
            if self.sock:
                self.sock.settimeout(None)

    def get_topics(self) -> list[tuple[str, str]]:
        self.connect()
        if self.sock:
            try:
                # Send the service call request
                self.send({
                    "op": "call_service",
                    "service": "/rosapi/topics",
                    "id": "get_topics_request_1"
                }, reconnect=False)
                
                # Receive the response
                response = self._receive_frame()
                print(f"[WebSocket] Received response: {response}")
                
                if response:
                    data = json.loads(response)
                    if "values" in data:
                        topics = data["values"].get("topics", [])
                        types = data["values"].get("types", [])
                        if topics and types and len(topics) == len(types):
                            return list(zip(topics, types))
                        else:
                            print("[WebSocket] Mismatch in topics and types length")
            except json.JSONDecodeError as e:
                print(f"[WebSocket] JSON decode error: {e}")
            except Exception as e:
                print(f"[WebSocket] Error: {e}")
        return []

    def close(self):
        if self.sock:
            try:
                self.sock.close()
                print("[WebSocket] Closed")
            except Exception as e:
                print(f"[WebSocket] Close error: {e}")
            finally:
                self.sock = None
