FROM ros:jazzy

RUN apt-get update && apt-get install -y \
    python3-pip \
    nano \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install pyTelegramBotAPI --break-system-packages

WORKDIR /ws

CMD ["bash"]
