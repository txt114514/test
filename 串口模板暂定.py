import asyncio
import serial
import serial_asyncio
import time

class SerialAsync_t(asyncio.Protocol):
    def __init__(self, port, baudrate, callback=None, reconnect_interval=2):
        self.port = port
        self.baudrate = baudrate
        self.callback = callback
        self.reconnect_interval = reconnect_interval

        self.transport = None
        self.loop = asyncio.get_event_loop()

        self.buffer = bytearray()       # 缓冲区
        self.queue = asyncio.Queue()    # 异步数据队列

    def connection_made(self, transport):
        self.transport = transport
        print(f"\033[92m[INFO] Serial connected: {self.port} @ {self.baudrate}\033[0m")

    def data_received(self, data): # 接收数据时存入队列
        self.queue.put_nowait(data)  

    def connection_lost(self, exc):
        print(f"\033[91m[WARNING] Serial connection lost: {exc}\033[0m")
        self.transport = None
        self.loop.create_task(self.reconnect())# 自动重连


    def write(self, data: bytes):
        if self.transport:
            self.transport.write(data)
        else:
            print("\033[91m[WARNING] Cannot write, serial not connected.\033[0m")

    async def reconnect(self):
        """循环重连"""
        while self.transport is None:
            try:
                await serial_asyncio.create_serial_connection(
                    self.loop,
                    lambda: self,
                    self.port,
                    baudrate=self.baudrate
                )
                print("\033[92m[INFO] Reconnected successfully.\033[0m")
                break
            except serial.SerialException as e:
                print(f"\033[91m[WARNING] Reconnect failed: {e}\033[0m")
                await asyncio.sleep(self.reconnect_interval)

    def start(self):
        # 启动重连任务
        self.loop.create_task(self.reconnect())


async def data_callback(serial_async):# 处理队列中的数据
    while True:
        data = await serial_async.queue.get()  # 取出队列中的一条数据并移除
        ts = time.time()
        await dispense(data)
        print(f"[DBG] recv {len(data)} bytes at {ts:.6f}: {data.hex()}")
        serial_async.queue.task_done()  # 标记已处理
async def dispense(data):
    await asyncio.sleep(0.001)  # 模拟处理时间
    print("DATA_OK")
    return data


def main():
    port = '/dev/tnt1'  # 替换为实际串口
    baudrate = 115200

    serial_async = SerialAsync_t(port, baudrate)
    serial_async.start()

    asyncio.get_event_loop().create_task(data_callback(serial_async))# 启动数据处理协程
    try:
        asyncio.get_event_loop().run_forever()
    except KeyboardInterrupt:
        print("\033[93m[INFO] Stopping serial listener...\033[0m")


if __name__ == "__main__":
    main()