graph TD
    A[레이더 센서 1] --> B[데이터 수집]
    B --> C[PTP 시간 동기화]
    C --> D[프레임 생성]
    D --> E[UDP 패킷 생성]
    E --> F[UDP 전송]

    G[마스터 노드] --> H[UDP 수신]
    H --> I[프레임 조립]
    I --> J[Radar Scan Message 생성]
    J --> K[ROS2 Publish]
    I --> L[Point Cloud2 Message 변환]
    L --> M[ROS2 Publish]

    F --> H