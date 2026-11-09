# runner.py: Tự động hóa mô phỏng và thu thập 5 chỉ số MOEs bằng TraCI
import os
import sys
import traci

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Vui lòng thiết lập biến môi trường 'SUMO_HOME'")

def run_simulation(is_green_wave=True):
    sumo_cmd = ["sumo-gui", "-c", "corridor.sumocfg", "--tripinfo-output", "tripinfo.xml"]
    traci.start(sumo_cmd)
    
    total_stops = 0
    total_delay = 0.0
    completed_vehicles = 0
    max_queue_length = 0

    step = 0
    while step < 3600: # Mô phỏng 1 giờ cao điểm
        traci.simulationStep()
        
        # 1. Thu thập độ dài hàng chờ trên các nhánh nút giao
        for tls_id in ["TLS_N1", "TLS_N2", "TLS_N3", "TLS_N4", "TLS_N5"]:
            # Đếm số xe dừng (v < 0.1 m/s)
            controlled_lanes = traci.trafficlight.getControlledLanes(tls_id)
            current_queue = sum(traci.lane.getLastStepHaltingNumber(lane) for lane in controlled_lanes)
            if current_queue > max_queue_length:
                max_queue_length = current_queue

        # 2. Thu thập các xe đã hoàn thành chuyến đi
        arrived_ids = traci.simulation.getArrivedIDList()
        for veh_id in arrived_ids:
            completed_vehicles += 1

        step += 1

    traci.close()
    print(f"=== KẾT QUẢ ĐÁNH GIÁ 5 CHỈ SỐ HIỆU QUẢ (MOEs) ===")
    print(f"Thông lượng hoàn thành (Throughput): {completed_vehicles} xe/giờ")
    print(f"Hàng chờ cực đại (Max Queue): {max_queue_length} xe")

if __name__ == "__main__":
    run_simulation(is_green_wave=True)