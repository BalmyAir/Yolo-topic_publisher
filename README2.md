# 탁구공 인식 모델
------------
## 개요
CVAT을 활용하여 50장의 Train 데이터셋과 15장의 validation 데이터셋을 생성하여 batch 8로 하여 200 epoch 학습하였습니다.

## 명령어

    python train.py --img 640 --batch 8 --epochs 200 --data /workspace/src/yolov3/data/table_tennis.yaml --weights /workspace/src/yolov3/pretrain/yolov3.pt

