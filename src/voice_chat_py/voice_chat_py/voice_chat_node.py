import os
import sys
import threading
import queue
import wave
import datetime
from pathlib import Path
import time  # 引入时间模块

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

import pyaudio
import speech_recognition as sr
from openai import OpenAI
from pydub import AudioSegment
import pygame  # 导入pygame用于播放音频

# =============== 1. 配置信息 ===============
API_KEY = "sk-nftsgxpdsrdnnbgdzralzbewmhiylqkhrjthtugvlbyqaiph"  # 替换成你的API Key
BASE_URL = "https://api.siliconflow.cn/v1"

# 大模型（聊天）
MODEL_CHAT = "deepseek-ai/DeepSeek-V2.5"
# TTS 模型
MODEL_VOICE = "FunAudioLLM/CosyVoice2-0.5B"
VOICE_NAME = "FunAudioLLM/CosyVoice2-0.5B:david"

# 初始化 OpenAI 客户端
client = OpenAI(api_key=API_KEY, base_url=BASE_URL)

# 录音相关配置
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000  # 16kHz

# 确保存音频的文件夹存在
RECORD_FOLDER = Path("mp3_record")
if not RECORD_FOLDER.exists():
    RECORD_FOLDER.mkdir(parents=True, exist_ok=True)


# =============== 2. 多线程录音类 ===============
class AudioRecorder(threading.Thread):
    """
    用多线程方式从麦克风持续读音频帧，直到 stop_event 被设置为 True，
    将录下的音频帧放入 frames_queue。加入时间限制，超过20秒停止录音。
    """
    def __init__(self, stop_event: threading.Event, frames_queue: queue.Queue, device_index=None):
        super().__init__()
        self.stop_event = stop_event
        self.frames_queue = frames_queue
        self.device_index = device_index
        self.start_time = None  # 记录开始时间

    def run(self):
        p = pyaudio.PyAudio()
        stream = p.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=RATE,
            input=True,
            frames_per_buffer=CHUNK,
            input_device_index=self.device_index
        )
        print("开始录音...")
        self.start_time = time.time()  # 记录开始时间
        while not self.stop_event.is_set():
            # 检查录音时间是否超过20秒
            if time.time() - self.start_time > 20:
                print("录音时间已超过20秒，自动停止")
                self.stop_event.set()  # 设置 stop_event 以停止录音
            data = stream.read(CHUNK)
            self.frames_queue.put(data)
        
        stream.stop_stream()
        stream.close()
        p.terminate()
        print("录音结束")


# =============== 3. 录音数据转文字 ===============
def speech_to_text(frames, sample_width):
    """
    用 SpeechRecognition 对录音数据进行识别（调用谷歌引擎）。
    frames：音频帧列表；sample_width：每个采样字节数（paInt16 为 2）
    """
    r = sr.Recognizer()
    audio_data = b"".join(frames)
    audio = sr.AudioData(audio_data, RATE, sample_width)
    try:
        text = r.recognize_google(audio, language='zh-CN')
        return text
    except sr.UnknownValueError:
        return None
    except sr.RequestError as e:
        return f"语音识别服务出错: {e}"


# =============== 4. 保存用户语音为 MP3 ===============
def save_input_audio(frames, sample_width) -> Path:
    """
    将录下的 frames 转为临时 WAV，再转换成 MP3 文件，并返回 MP3 文件路径。
    文件名带时间戳，保存在 mp3_record 目录中。
    """
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    wav_path = RECORD_FOLDER / f"temp_input_{timestamp}.wav"
    with wave.open(str(wav_path), 'wb') as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(sample_width)
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
    mp3_path = RECORD_FOLDER / f"input_{timestamp}.mp3"
    audio_seg = AudioSegment.from_wav(wav_path)
    audio_seg.export(mp3_path, format="mp3")
    if wav_path.exists():
        wav_path.unlink()
    return mp3_path


# =============== 5. 调用大语言模型，获取回复 ===============
def get_model_response(user_text: str):
    """
    调用 SiliconCloud 的大模型接口（OpenAI 兼容），返回文本回复
    """
    try:
        response = client.chat.completions.create(
            model=MODEL_CHAT,
            messages=[
                {"role": "system", "content": "你毕加索公司的四足机器狗，叫做来福。你用中文回答问题，有趣又调皮，回答很简短，喜欢汪汪叫。"},
                {"role": "user", "content": user_text}
            ],
            temperature=0.7,
            max_tokens=256,
            stream=False
        )
        if response and hasattr(response, 'choices'):
            return response.choices[0].message.content
        else:
            return "对不起，模型没有返回有效结果。"
    except Exception as e:
        return f"调用模型接口出现错误: {str(e)}"


# =============== 6. 调用 SiliconCloud TTS 生成语音并播放 ===============
def speak_text_siliconcloud(text: str):
    """
    调用 SiliconCloud 提供的 TTS 接口生成中文语音（MP3），
    播放生成的语音并将音频保存到 mp3_record 目录中。
    """
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_mp3 = RECORD_FOLDER / f"output_{timestamp}.mp3"
    try:
        with client.audio.speech.with_streaming_response.create(
            model=MODEL_VOICE,
            voice=VOICE_NAME,
            input=text,
            response_format="mp3"
        ) as response:
            response.stream_to_file(output_mp3)

        # 初始化 pygame.mixer 模块
        pygame.mixer.init()

        # 加载并播放音频
        pygame.mixer.music.load(str(output_mp3))
        pygame.mixer.music.play()

        # 等待音频播放完毕
        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)

    except Exception as e:
        print(f"调用 SiliconCloud 语音合成失败: {str(e)}")


# =============== 7. ROS2 节点：语音对话 ===============
class VoiceChatNode(Node):
    def __init__(self):
        super().__init__('voice_chat_node')
        # 订阅 "voice_chat/control" 话题，消息类型 std_msgs/String
        self.subscription = self.create_subscription(
            String,
            'SMXFE/VoiceCmd',
            self.control_callback,
            10
        )

        self.publisher = self.create_publisher(Float64MultiArray, 'SMXFE/JoystickCmd', 10)

        self.get_logger().info("VoiceChatNode 已启动")
        self.recording = False
        self.frames_queue = None
        self.stop_event = None
        self.recorder = None



    def control_callback(self, msg: String):
        command = msg.data.strip().lower()
        if command == "start":
            if not self.recording:
                self.get_logger().info("开始录音")
                self.frames_queue = queue.Queue()
                self.stop_event = threading.Event()
                self.recorder = AudioRecorder(self.stop_event, self.frames_queue)
                self.recorder.start()
                self.recording = True
            else:
                self.get_logger().warn("当前已在录音状态")
        elif command == "stop":
            if self.recording:
                self.get_logger().info("停止录音")
                self.stop_event.set()
                self.recorder.join()
                self.recording = False
                frames = []
                while not self.frames_queue.empty():
                    frames.append(self.frames_queue.get())
                sample_width = 2  # 对于 paInt16，每个样本2字节
                input_mp3_path = save_input_audio(frames, sample_width)
                self.get_logger().info(f"保存用户语音: {input_mp3_path}")
                text = speech_to_text(frames, sample_width)
                if not text:
                    self.get_logger().error("未识别或识别出错")
                    return
                
                if len(text) < 5:
                    if any(word in text for word in ["坐", "作", "做", "座"]):
                        self.get_logger().info("检测到命令: 坐")
                        self.publish_joystick_cmd(22110000, 0, 0)
                        return
                    elif any(word in text for word in ["趴", "爬", "怕"]):
                        self.get_logger().info("检测到命令: 趴")
                        self.publish_joystick_cmd(25110000, 0, 0)
                        return
                    elif any(word in text for word in ["占", "站", "战", "绽"]):
                        self.get_logger().info("检测到命令: 站")
                        self.publish_joystick_cmd(25100000, 0, 0)
                        return
                
                self.get_logger().info(f"识别结果: {text}")
                response_text = get_model_response(text)
                self.get_logger().info(f"模型回复: {response_text}")
                speak_text_siliconcloud(response_text)
            else:
                self.get_logger().warn("当前未在录音状态")
        else:
            self.get_logger().info(f"未知命令: {command}")
    

    def publish_joystick_cmd(self, Action, Value1, Value2):
        """
        发布 SMXFE/JoystickCmd 消息，包含三个 double 类型的值。
        """
        msg = Float64MultiArray()
        msg.data = [float(Action), float(Value1), float(Value2)]  # 将三个数值添加到消息中
        self.publisher.publish(msg)
        self.get_logger().info(f"发布消息 SMXFE/JoystickCmd: {msg.data}")


def main():
    print("Starting voice chat node...")
    # 其他启动逻辑

    rclpy.init()
    node = VoiceChatNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
