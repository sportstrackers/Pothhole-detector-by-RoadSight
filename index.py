
import sys
import os
import io


sys.path.insert(0, 'ultralytics')
sys.path.insert(0, 'pynmea2')

import cv2
import csv
from datetime import datetime
from ultralytics import YOLO


try:
    import serial
    import pynmea2
    HAS_GPS = True
except ImportError:
    HAS_GPS = False

class PotholeDetector:
    def __init__(self, model_path="my_model.pt"):
        
        self.model = YOLO(model_path)
        self.detection_count = 0
        self.gps_coords = [0.0, 0.0]
        
        
        self.gps_serial = None
        self.gps_sio = None
        if HAS_GPS:
            try:
                self.gps_serial = serial.Serial('/dev/ttyAMA0', 9600, timeout=5.0)
                self.gps_sio = io.TextIOWrapper(io.BufferedRWPair(self.gps_serial, self.gps_serial))
            except:
                pass
        
        
        if not os.path.exists('detections.csv'):
            with open('detections.csv', 'w', newline='') as f:
                csv.writer(f).writerow(['ID', 'Time', 'Lat', 'Lon', 'Conf', 'Image'])

    def get_gps(self):
    
        if not self.gps_sio:
            return
        
        try:
            line = self.gps_sio.readline()
            msg = pynmea2.parse(line)
            if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
                if msg.latitude and msg.longitude:
                    self.gps_coords = [msg.latitude, msg.longitude]
        except pynmea2.ParseError:
            pass  
        except:
            pass

    def run_detection(self):

        cap = cv2.VideoCapture(0)
        print("Pothole Detection Started")
        
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                
                
                self.get_gps()
                
                
                results = self.model(frame, conf=0.5, verbose=False)
                
                if len(results[0].boxes) > 0:
                    self.detection_count += 1
                    
                    
                    annotated_frame = results[0].plot()
                    filename = f"pothole_{self.detection_count:04d}.jpg"
                    cv2.imwrite(filename, annotated_frame)
                    
                    
                    confidence = float(results[0].boxes.conf.max())
                    with open('detections.csv', 'a', newline='') as f:
                        csv.writer(f).writerow([
                            self.detection_count,
                            datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                            self.gps_coords[0],
                            self.gps_coords[1],
                            f"{confidence:.3f}",
                            filename
                        ])
                    
                    print(f"Pothole {self.detection_count} detected!")
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        except KeyboardInterrupt:
            print("Stopped")
        finally:
            cap.release()
            cv2.destroyAllWindows()
            if self.gps_serial:
                self.gps_serial.close()

if __name__ == "__main__":
    detector = PotholeDetector()
    detector.run_detection()


def run():
    
      OUTPUT_DIR.mkdir(exist_ok=True)
    
   create_csv()
    
    print('Loading model...')
    model = YOLO(MODEL_PATH)
    
    print('Setting up GPS...')
    gps_serial = serial.Serial('/dev/ttyAMA0', baudrate=9600, timeout=1)
    
    
    print('Setting up camera...')
    camera = Picamera2()
    camera_config = camera.create_preview_configuration(
        main={'size': (640, 480), 'format': 'RGB888'}
    )
    camera.configure(camera_config)
    camera.start()
    time.sleep(2)  
    
    print('\nPothole Detection Started!')
    print('Press Ctrl+C to stop\n')
    
    while True:
        
        read_gps(gps_serial)
        
        
        frame = capture_frame(camera)
        
    
      results = detect_potholes(model, frame)
        
        
        if len(results.boxes) > 0:
            save_detection(frame, results)
        
        
        time.sleep(0.1)

if __name__ == '__main__':
    run()
