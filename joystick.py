import tkinter as tk
import serial
import time
import threading
import math

# 初始化串口通信 (根據你的 Arduino 端口修改)
ser = serial.Serial('COM6', 115200)  # 設置你的 Arduino 串口端口
time.sleep(2)  # 等待串口準備

# 創建主窗口
root = tk.Tk()
root.title("Joystick and Lever Control")

# 搖桿初始位置
joystick_x = 0
joystick_y = 0

# 拉桿初始高度
lever_value = 50

# 搖桿是否被拖動
joystick_active = False

# Demo 控制變量
circle_demo_active = False
theta = 0  # 用於記錄圓的角度
radius = 0.5  # 圓的初始半徑
speed = 0.3  # 圓的初始速度

# 創建一個用來顯示回傳角度的文本區域
output_text = tk.Text(root, height=10, width=50)
output_text.grid(row=3, column=0, columnspan=2)

# 更新搖桿位置並發送數據到 Arduino
def update_joystick(event):
    global joystick_x, joystick_y, joystick_active, circle_demo_active
    circle_demo_active = False  # 禁止圓形 Demo
    joystick_active = True
    # 將鼠標位置轉換為 -1.0 到 1.0 的範圍
    joystick_x = (event.x - 150) / 100.0  # 中心點是 (150, 150)
    joystick_y = -(event.y - 150) / 100.0  # Y 軸反向
    joystick_x = max(-1.0, min(1.0, joystick_x))  # 限制範圍
    joystick_y = max(-1.0, min(1.0, joystick_y))  # 限制範圍
    send_data()

# 停止拖動時，搖桿自動回到中心
def release_joystick(event):
    global joystick_active
    joystick_active = False

# 每一幀讓搖桿平滑回到中心
def return_to_center():
    global joystick_x, joystick_y
    if not joystick_active and not circle_demo_active:
        joystick_x *= 0.1  # 緩慢返回中心
        joystick_y *= 0.1
        if abs(joystick_x) < 0.01:
            joystick_x = 0
        if abs(joystick_y) < 0.01:
            joystick_y = 0
    update_joystick_position()
    send_data()
    root.after(50, return_to_center)

# 更新搖桿在畫布中的位置
def update_joystick_position():
    canvas.coords(joystick, 150 + joystick_x * 100 - 10, 150 - joystick_y * 100 - 10,
                  150 + joystick_x * 100 + 10, 150 - joystick_y * 100 + 10)

# 更新拉桿高度並發送數據到 Arduino
def update_lever(event):
    global lever_value
    lever_value = 100 - int(event.y / 3)  # 轉換為 0 到 100
    lever_value = max(0, min(100, lever_value))  # 限制範圍
    send_data()

# 發送搖桿和拉桿數據到 Arduino
def send_data():
    joystick_command = f"joystick: {joystick_x:.2f},{joystick_y:.2f}\n"
    lever_command = f"lever: {lever_value}\n"
    ser.write(joystick_command.encode())
    ser.write(lever_command.encode())
    print(joystick_command.strip(), lever_command.strip())

# 啟動圓形 Demo
def start_circle_demo():
    global circle_demo_active, theta, radius, speed
    circle_demo_active = True
    theta = 0  # 重置角度
    try:
        # 從輸入框讀取半徑和速度
        radius = float(radius_entry.get())
        speed = float(speed_entry.get())
    except ValueError:
        print("請輸入有效的數字")
        return
    update_circle_demo()

# 更新圓形 Demo (圓周運動)
def update_circle_demo():
    global joystick_x, joystick_y, theta
    if circle_demo_active:
        # 使用輸入的半徑和速度
        joystick_x = radius * math.cos(theta)
        joystick_y = radius * math.sin(theta)
        theta += speed  # 根據速度調整角度
        if theta >= 2 * math.pi:
            theta = 0  # 角度歸 0

        update_joystick_position()
        send_data()

        # 每 50 毫秒更新一次
        root.after(50, update_circle_demo)

# 創建畫布用於搖桿控制
canvas = tk.Canvas(root, width=300, height=300, bg="lightgray")
canvas.grid(row=0, column=0, padx=20, pady=20)

# 繪製搖桿區域
canvas.create_oval(50, 50, 250, 250, fill="white")
joystick = canvas.create_oval(140, 140, 160, 160, fill="blue")

# 搖桿事件綁定
canvas.bind("<B1-Motion>", update_joystick)  # 鼠標拖動事件
canvas.bind("<ButtonRelease-1>", release_joystick)  # 鼠標釋放事件

# 創建畫布用於拉桿控制
lever_canvas = tk.Canvas(root, width=50, height=300, bg="lightgray")
lever_canvas.grid(row=0, column=1, padx=20, pady=20)

# 繪製拉桿區域
lever_canvas.create_rectangle(20, 50, 40, 250, fill="white")
lever = lever_canvas.create_rectangle(20, 150, 40, 170, fill="red")

# 拉桿事件綁定
lever_canvas.bind("<B1-Motion>", update_lever)

# 更新拉桿位置
def update_lever_position():
    lever_canvas.coords(lever, 20, 250 - lever_value * 2, 40, 270 - lever_value * 2)

# 周期性更新函數
def periodic_update():
    update_lever_position()
    root.after(100, periodic_update)

# 接收 Arduino 回傳訊息的函數
def receive_data():
    while True:
        if ser.in_waiting > 0:
            try:
                data = ser.readline().decode('utf-8').strip()  # 接收數據
                output_text.insert(tk.END, data + "\n")  # 在界面上顯示回傳數據
                output_text.see(tk.END)  # 滾動到最新行
            except Exception as e:
                print(f"Error: {e}")

# 創建一個新線程來接收 Arduino 回傳訊息
thread = threading.Thread(target=receive_data)
thread.daemon = True  # 守護進程，程序關閉時自動結束
thread.start()

# 創建 "圓形 Demo" 按鈕
circle_demo_button = tk.Button(root, text="圓形 Demo", command=start_circle_demo)
circle_demo_button.grid(row=1, column=0, padx=10, pady=10)

# 創建半徑輸入框
radius_label = tk.Label(root, text="半徑:")
radius_label.grid(row=1, column=1)
radius_entry = tk.Entry(root)
radius_entry.grid(row=1, column=2)
radius_entry.insert(0, "0.5")  # 預設值

# 創建速度輸入框
speed_label = tk.Label(root, text="速度:")
speed_label.grid(row=2, column=1)
speed_entry = tk.Entry(root)
speed_entry.grid(row=2, column=2)
speed_entry.insert(0, "0.2")  # 預設值

# 開始週期性更新
periodic_update()
return_to_center()  # 開始搖桿回正功能

# 啟動主事件循環
root.mainloop()

# 關閉串口
ser.close()
