import serial
import csv
import sys

PORT = 'COM26'
BAUDRATE = 460800
OUTPUT_FILE = 'ident_data.csv'


def main():
    print(f"正在打开串口 {PORT} @ {BAUDRATE}...")
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1.0)
    except serial.SerialException as e:
        print(f"无法打开串口: {e}")
        sys.exit(1)

    print(f"已连接串口。开始将数据记录到 {OUTPUT_FILE} ...")
    print("按 Ctrl+C 停止记录并保存文件。")

    try:
        with open(OUTPUT_FILE, mode='w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # 写入辨识脚本需要的表头顺序
            writer.writerow(['time', 'q1', 'q2', 'dq1', 'dq2', 'tau1', 'tau2'])

            # 刷新一下串口旧数据
            ser.reset_input_buffer()

            count = 0
            while True:
                line = ser.readline()
                if not line:
                    continue

                try:
                    # 解码并去除两端空白符
                    line_str = line.decode('utf-8').strip()
                    # 分割数据
                    data = line_str.split(',')

                    if len(data) == 7:
                        # 解析STM32发来的数据顺序：
                        # STM32 raw CSV order:
                        # 0: timestamp_ms
                        # 1: tau_yaw
                        # 2: q_yaw
                        # 3: dq_yaw
                        # 4: tau_pitch
                        # 5: q_pitch
                        # 6: dq_pitch

                        t_ms = float(data[0])
                        time_s = t_ms / 1000.0  # 毫秒转秒

                        tau1 = float(data[1])
                        q1 = float(data[2])
                        dq1 = float(data[3])
                        tau2 = float(data[4])
                        q2 = float(data[5])
                        dq2 = float(data[6])

                        # 按照列名表头顺序写入 CSV ['time', 'q1', 'q2', 'dq1', 'dq2', 'tau1', 'tau2']
                        writer.writerow([time_s, q1, q2, dq1, dq2, tau1, tau2])

                        count += 1
                        if count % 100 == 0:
                            print(f"已记录 {count} 条数据列...")

                except ValueError:
                    # 处理偶发的串口乱码
                    print(f"解析错误，跳过该行: {line_str}")
                    continue
                except UnicodeDecodeError:
                    continue

    except KeyboardInterrupt:
        print("\n\nCtrl+C 已按下，停止记录。")
    finally:
        ser.close()
        print(f"串口已关闭，数据已安全保存至 {OUTPUT_FILE}。")


if __name__ == '__main__':
    main()
