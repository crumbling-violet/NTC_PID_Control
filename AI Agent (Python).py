import serial
import time
import json
import re
from openai import OpenAI

# ================= 配置参数 =================
SERIAL_PORT = 'COM3'  # 替换为你的实际串口号 (Linux下可能是 /dev/ttyUSB0)
BAUD_RATE = 115200
TARGET_TEMP = 45.0    # 目标温度
TEST_DURATION = 30    # 每次测试参数收集数据的时长（秒）

# 初始化大模型客户端 (以兼容 OpenAI API 格式为例)
client = OpenAI(
    api_key="YOUR_API_KEY", 
    base_url="https://api.openai.com/v1" # 如果使用其他模型，替换为对应 Base URL
)

# 初始 PID 参数
current_pid = {"Kp": 1.0, "Ki": 0.1, "Kd": 0.05}

def send_pid_to_hardware(ser, pid_dict):
    """将 PID 参数打包成字符串下发给 STM32"""
    # 格式例如: P:1.20,I:0.15,D:0.08\n
    cmd = f"P:{pid_dict['Kp']:.3f},I:{pid_dict['Ki']:.3f},D:{pid_dict['Kd']:.3f}\n"
    ser.write(cmd.encode('utf-8'))
    print(f"[硬件下发] 已发送新参数: {cmd.strip()}")

def analyze_and_tune(history_data, target, current_pid):
    """调用大模型 Agent 进行长链推理调参"""
    print("\n[AI Agent] 正在分析温度曲线并计算新参数...")
    
    prompt = f"""
    你是一个专业的嵌入式硬件 PID 调参专家。
    当前目标温度：{target}°C。
    当前使用的 PID 参数为：Kp={current_pid['Kp']}, Ki={current_pid['Ki']}, Kd={current_pid['Kd']}。
    
    以下是过去 {TEST_DURATION} 秒内硬件回传的温度时间序列数据（每秒采样一次）：
    {history_data}
    
    请分析该温度曲线（评估上升时间、超调量、稳态误差和振荡情况），并给出下一轮的 PID 参数。
    
    限制条件：
    1. Kp 的合理范围是 0.1 到 20.0
    2. Ki 的合理范围是 0.0 到 5.0
    3. Kd 的合理范围是 0.0 到 10.0
    
    请严格以 JSON 格式输出，不要包含任何 Markdown 标记（如 ```json），只需输出 JSON 本身。格式如下：
    {{
        "analysis": "这里简述你的分析过程和调参理由",
        "Kp": 1.5,
        "Ki": 0.2,
        "Kd": 0.1
    }}
    """
    
    try:
        response = client.chat.completions.create(
            model="gpt-4o", # 或其他具备强逻辑推理能力的模型
            messages=[{"role": "user", "content": prompt}],
            temperature=0.2
        )
        
        result_text = response.choices[0].message.content.strip()
        # 清理可能存在的 markdown 格式
        result_text = re.sub(r"```json\n?|```", "", result_text).strip()
        
        new_params = json.loads(result_text)
        print(f"[AI Agent] 分析结论: {new_params.get('analysis')}")
        return {
            "Kp": float(new_params['Kp']),
            "Ki": float(new_params['Ki']),
            "Kd": float(new_params['Kd'])
        }
    except Exception as e:
        print(f"[AI Agent] 推理失败或 JSON 解析错误: {e}")
        return current_pid # 失败则维持原参数

def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2) # 等待串口稳定
    
    global current_pid
    iteration = 1
    
    while True:
        print(f"\n========== 第 {iteration} 轮调参 ==========")
        send_pid_to_hardware(ser, current_pid)
        
        history_data = []
        start_time = time.time()
        
        # 收集一轮测试数据
        while time.time() - start_time < TEST_DURATION:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8').strip()
                if line.startswith("T:"):
                    # 假设 STM32 上报格式为 "T:43.5\n"
                    try:
                        temp = float(line.split(":")[1])
                        history_data.append(temp)
                        print(f"当前温度: {temp}°C")
                    except ValueError:
                        pass
            time.sleep(0.5)
            
        # 调用 Agent 进行分析
        current_pid = analyze_and_tune(history_data, TARGET_TEMP, current_pid)
        iteration += 1

if __name__ == "__main__":
    main()