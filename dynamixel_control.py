import os
import json
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import tkinter as tk
from tkinter import ttk
from dynamixel_sdk import *

class MultiJointDynamixelController:
    def __init__(self, port="COM3", baudrate=1000000):
        self.port = port
        self.baudrate = baudrate
        
        self.portHandler = PortHandler(self.port)
        self.packetHandler = PacketHandler(2.0) 
        
        self.groupSyncRead = GroupSyncRead(
            self.portHandler,
            self.packetHandler,
            132,  
            4     
        )
        
        self.groupSyncWrite = GroupSyncWrite(
            self.portHandler,
            self.packetHandler,
            116, 
            4    
        )
        
        self.ADDR_OPERATING_MODE = 11
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_PROFILE_VELOCITY = 112
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132
        
        try:
            if not self.portHandler.openPort():
                raise Exception(f"Failed to open port {self.port}")
            
            if not self.portHandler.setBaudRate(self.baudrate):
                raise Exception(f"Failed to set baudrate to {self.baudrate}")
                
            print(f"Dynamixel controller initialized on {self.port} at {self.baudrate} baud")
            
        except Exception as e:
            print(f"Error initializing controller: {e}")
            raise
    
    def setup_motors(self, motor_ids, velocity=1023):
        for motor_id in motor_ids:
            try:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                    self.portHandler, motor_id, self.ADDR_TORQUE_ENABLE, 0
                )
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"Failed to disable torque on motor {motor_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                    continue
                
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                    self.portHandler, motor_id, self.ADDR_OPERATING_MODE, 4
                )
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"Failed to set operating mode on motor {motor_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                    continue
                
                dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
                    self.portHandler, motor_id, self.ADDR_PROFILE_VELOCITY, velocity
                )
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"Failed to set velocity on motor {motor_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                    continue
                
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                    self.portHandler, motor_id, self.ADDR_TORQUE_ENABLE, 1
                )
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"Failed to enable torque on motor {motor_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                    continue
                
                self.groupSyncRead.addParam(motor_id)
                
                print(f"Motor ID {motor_id} setup complete (Extended Position Mode, Velocity: {velocity})")
                
            except Exception as e:
                print(f"Error setting up motor {motor_id}: {e}")
    
    def set_multiple_positions_simultaneously(self, motor_positions):

        self.groupSyncWrite.clearParam()
        
        for motor_id, position in motor_positions.items():
            position = max(-256000, min(256000, int(position)))
            
            position_bytes = [
                position & 0xFF,
                (position >> 8) & 0xFF,
                (position >> 16) & 0xFF,
                (position >> 24) & 0xFF
            ]
            
            dxl_addparam_result = self.groupSyncWrite.addParam(motor_id, position_bytes)
            if not dxl_addparam_result:
                print(f"Failed to add param for motor {motor_id}")
        
        dxl_comm_result = self.groupSyncWrite.txPacket()
        
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to send group sync write: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return False
        
        return True
    
    def read_positions(self, motor_ids):
        dxl_comm_result = self.groupSyncRead.txRxPacket()
        

        positions = {}
        
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to read positions: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        else:

            for motor_id in motor_ids:

                if self.groupSyncRead.isAvailable(motor_id, self.ADDR_PRESENT_POSITION, 4):

                    position = self.groupSyncRead.getData(motor_id, self.ADDR_PRESENT_POSITION, 4)
                    
                    if position > 2147483647: 
                        position = position - 4294967296 
                    
                    positions[motor_id] = position
                else:
                    print(f"Failed to get position data from motor ID {motor_id}")
                    positions[motor_id] = None
        
        return positions
    
    def close(self):
        self.portHandler.closePort()


class SimpleAnimationPlayer:
    def __init__(self, controller, animation_file):
        self.controller = controller
        
        try:
            with open(animation_file, 'r') as f:
                self.animation_data = json.load(f)
            
            self.metadata = self.animation_data["metadata"]
            self.motors = self.animation_data["motors"]
            self.fps = self.metadata["fps"]
            
            self.motor_ids = [self.motors[joint]["motor_id"] for joint in self.motors]
            
            self.base_positions = {}
            self.animation_offsets = {}
            
            print(f"Motor IDs: {self.motor_ids}")
            print(f"Duration: {self.metadata['duration_seconds']} seconds")
            
        except Exception as e:
            print(f"Error loading animation file: {e}")
            raise
    
    def calculate_animation_offsets(self):
        frames = self.animation_data["frames"]
        
        first_frame = frames[0]
        motor_base_values = {}
        
        for joint_name, joint_data in first_frame["joints"].items():
            motor_id = joint_data["motor_id"]
            base_value = joint_data["dynamixel_position"]
            motor_base_values[motor_id] = base_value
        
        self.animation_offsets = {}
        
        for motor_id in self.motor_ids:
            self.animation_offsets[motor_id] = []
            base_value = motor_base_values[motor_id]
            
            for frame in frames:
                for joint_name, joint_data in frame["joints"].items():
                    if joint_data["motor_id"] == motor_id:
                        current_value = joint_data["dynamixel_position"]
                        offset = current_value - base_value
                        self.animation_offsets[motor_id].append(offset)
                        break
        
        print("\n=== Animation Information ===")
        
        for motor_id in self.motor_ids:
            offsets = self.animation_offsets[motor_id]
            min_offset = min(offsets)
            max_offset = max(offsets)
            range_offset = max_offset - min_offset
            
            print(f"Motor {motor_id}: Range {min_offset} ~ {max_offset} "
                  f"(Change: {range_offset} units, {range_offset*360/4096:.1f}°)")
    
    def set_base_positions(self):
        
        current_positions = self.controller.read_positions(self.motor_ids)
        
        for motor_id in self.motor_ids:
            current_pos = current_positions.get(motor_id)
            if current_pos is not None:
                self.base_positions[motor_id] = current_pos
                current_angle = current_pos * 360 / 4096
                print(f"Motor {motor_id}: Base position {current_pos} units ({current_angle:.1f}°)")
            else:
                self.base_positions[motor_id] = 0
        
        print("Base position set Complete")
    
    def get_relative_position(self, motor_id, frame_index):
        if motor_id not in self.base_positions or motor_id not in self.animation_offsets:
            return 0
        
        base_pos = self.base_positions[motor_id]
        offset = self.animation_offsets[motor_id][frame_index]
        
        absolute_pos = base_pos + offset
        
        absolute_pos = max(-256000, min(256000, int(absolute_pos)))
        
        return absolute_pos
    
    def setup(self, skip_motor_init=False):
        if not skip_motor_init:
            self.controller.setup_motors(self.motor_ids, velocity=1023)
        
        self.calculate_animation_offsets()
        self.set_base_positions()
    
    def play_animation(self, interrupt_check=None, animation_state=None):
        frames = self.animation_data["frames"]
        speed_factor = 1.0
        
        times = []
        target_positions = {motor_id: [] for motor_id in self.motor_ids}
        actual_positions = {motor_id: [] for motor_id in self.motor_ids}
        position_errors = {motor_id: [] for motor_id in self.motor_ids}
        
        if animation_state:
            animation_state.update_progress(0, len(frames))
        
        print(f"\n=== Play Animation ===")
        print(f"Total Frames: {len(frames)}")
        
        start_time = time.time()
        
        try:
            for i, frame in enumerate(frames):
                if animation_state:
                    animation_state.update_progress(i + 1, len(frames))
                
                if interrupt_check and callable(interrupt_check):
                    if interrupt_check():
                        print(f"\n Animation Stopped (Frame {i+1}/{len(frames)})")
                        break
                current_time = (time.time() - start_time) * speed_factor
                target_time = frame["time"] / speed_factor
                
                if current_time < target_time:
                    sleep_time = target_time - current_time
                    if sleep_time > 0.1:
                        sleep_steps = int(sleep_time / 0.05)
                        for step in range(sleep_steps):
                            if interrupt_check and callable(interrupt_check) and interrupt_check():
                                return
                            time.sleep(0.05)
                        remaining_time = sleep_time - (sleep_steps * 0.05)
                        if remaining_time > 0:
                            time.sleep(remaining_time)
                    else:
                        time.sleep(sleep_time)
                
                motor_positions = {}
                for motor_id in self.motor_ids:
                    final_position = self.get_relative_position(motor_id, i)
                    motor_positions[motor_id] = final_position
                
                success = self.controller.set_multiple_positions_simultaneously(motor_positions)
                
                times.append(target_time)
                current_actual = self.controller.read_positions(self.motor_ids)
                
                for motor_id in self.motor_ids:
                    target_pos = motor_positions[motor_id]
                    actual_pos = current_actual.get(motor_id)
                    
                    target_positions[motor_id].append(target_pos)
                    actual_positions[motor_id].append(actual_pos)
                    
                    if actual_pos is not None:
                        error = abs(target_pos - actual_pos)
                        position_errors[motor_id].append(error)
                    else:
                        position_errors[motor_id].append(None)
                
                if i % 20 == 0:
                    motor_info = []
                    for motor_id in self.motor_ids:
                        base_pos = self.base_positions[motor_id]
                        offset = self.animation_offsets[motor_id][i]
                        final_pos = motor_positions[motor_id]
                        motor_info.append(f"M{motor_id}: {base_pos}+{offset}={final_pos}")
                    
                    print(f"Frame {i+1}/{len(frames)} | Time: {target_time:.2f}s | {' | '.join(motor_info)}")
                
                time.sleep(0.0005)
                
                if i % 30 == 0:
                    progress = (i + 1) / len(frames) * 100
                    print(f"Progress: {progress:.1f}%", end="\r")
            
            print(f"\n\n=== Animation Complete ===")
            
            print("\n=== Final Position ===")
            final_positions = self.controller.read_positions(self.motor_ids)
            
            for motor_id in self.motor_ids:
                base_pos = self.base_positions[motor_id]
                final_pos = final_positions.get(motor_id, 0)
                total_movement = final_pos - base_pos if final_pos is not None else 0
                
                print(f"Motor {motor_id}: {base_pos} → {final_pos} "
                      f"(Total Movement: {total_movement} units, {total_movement*360/4096:.1f}°)")
            
            self.plot_results(times, target_positions, actual_positions, position_errors)
            
        except KeyboardInterrupt:
            print("\n\nAniamation stopped by user.")
        except Exception as e:
            print(f"\nError occured: {e}")
    
    def plot_results(self, times, target_positions, actual_positions, position_errors=None):
        try:
            root = tk.Tk()
            root.title("Animation Results")
            root.geometry("1200x800")
            
            main_frame = ttk.Frame(root)
            main_frame.pack(fill=tk.BOTH, expand=1)
            
            canvas = tk.Canvas(main_frame)
            canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=1)
            
            scrollbar = ttk.Scrollbar(main_frame, orient=tk.VERTICAL, command=canvas.yview)
            scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
            
            canvas.configure(yscrollcommand=scrollbar.set)
            
            plot_frame = ttk.Frame(canvas)
            canvas.create_window((0, 0), window=plot_frame, anchor="nw")
            
            fig = Figure(figsize=(12, 4*len(self.motor_ids)), dpi=100)
            
            for i, motor_id in enumerate(self.motor_ids):
                ax1 = fig.add_subplot(len(self.motor_ids), 1, i + 1)
                ax1.plot(times, target_positions[motor_id], 'b-', label=f'Target', linewidth=2)
                
                actual_times = []
                actual_pos = []
                for t, pos in zip(times, actual_positions[motor_id]):
                    if pos is not None:
                        actual_times.append(t)
                        actual_pos.append(pos)
                
                if actual_times:
                    ax1.plot(actual_times, actual_pos, 'r-', label=f'Actual', linewidth=1)
                
                ax1.set_xlabel('Time (seconds)')
                ax1.set_ylabel('Position (units)')
                ax1.set_title(f'Motor ID {motor_id} - Position Control')
                ax1.legend()
                ax1.grid(True)
                
                ax1_deg = ax1.twinx()
                min_pos = min(target_positions[motor_id]) if target_positions[motor_id] else 0
                max_pos = max(target_positions[motor_id]) if target_positions[motor_id] else 4096
                ax1_deg.set_ylim(min_pos * 360 / 4096, max_pos * 360 / 4096)
                ax1_deg.set_ylabel('Angle (degrees)')
            
            fig.tight_layout()
            
            canvas_plot = FigureCanvasTkAgg(fig, master=plot_frame)
            canvas_plot.draw()
            canvas_plot.get_tk_widget().pack()
            
            plot_frame.update_idletasks()
            canvas.config(scrollregion=canvas.bbox("all"))
            
            def _on_mousewheel(event):
                canvas.yview_scroll(int(-1*(event.delta/120)), "units")
            
            canvas.bind_all("<MouseWheel>", _on_mousewheel)
            
            save_button = ttk.Button(root, text="Save Figure", 
                                    command=lambda: fig.savefig("animation_results.png", dpi=150, bbox_inches='tight'))
            save_button.pack(side=tk.BOTTOM, pady=5)
            
            root.mainloop()
            
        except Exception as e:
            print(f"Error plotting results: {e}")
    
    def play_animation_with_interrupt_check(self, animation_state):
        def check_interrupt():
            return animation_state.check_should_stop()
        
        self.play_animation(interrupt_check=check_interrupt)


if __name__ == "__main__":
    controller = None
    
    try:
        print("=== Animation Player ===")
        
        port = input("COM Port: ") or "COM3"
        
        controller = MultiJointDynamixelController(port=port)
        
        # Path 경로 설정
        animation_folder = "your path/애니메이션 폴더 경로/"
        
        first_animation = True
        
        while True:
            print("\n=== Select Animation File ===")
            
            file_input = input(f"Enter the name of animation file (Quit: 'q'): ").strip()
            
            if file_input.lower() in ['q', '']:
                print("Exiting Program.")
                break
            
            if not file_input.endswith('.json'):
                file_input += '.json'
            
            animation_file = os.path.join(animation_folder, file_input)
            
            if not os.path.exists(animation_file):
                print(f" Cannot find file: {animation_file}")
                print("Enter again")
                continue
            
            try:
                print(f"Loading file: {file_input}")
                player = SimpleAnimationPlayer(controller, animation_file)
                
                if first_animation:
                    player.setup()
                    first_animation = False
                else:
                    player.setup(skip_motor_init=True)
                
                input(f" '{file_input}'\nPress Enter to start")
                
                player.play_animation()
                
            except Exception as e:
                print(f"Error occured: {e}")
                print("Try other file.")
                continue
        
    except KeyboardInterrupt:
        print("\n\nProgram stopped by user.")
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        if controller is not None:
            controller.close()
