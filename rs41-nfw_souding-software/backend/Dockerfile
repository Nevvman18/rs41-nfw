FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3 python3-pip curl wget git unzip ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# arduino-cli binary (this step needs root to write into /usr/local/bin)
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=/usr/local/bin sh

# Non-root build user. uid 1000 matches the host user so the bind-mounted ./logs
# stays writable, and a compromised container never has root inside it.
RUN useradd -m -u 1000 -s /bin/bash builder

WORKDIR /app
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

COPY app.py .
COPY rs41_subframe.py .
COPY static/ ./static/

RUN mkdir -p /workspace /logs && chown -R builder:builder /workspace /logs /app

USER builder

# Install the STM32 board package as the build user, so it lives in the user's home
# directory. That home stays writable at runtime (the L412 variant is copied in on
# first fetch) without anything ever needing root.
# NOTE: the platform ID is "STMicroelectronics:stm32" (the package vendor name), NOT
# "STM32:stm32". Using the wrong ID gives the misleading "Platform not found" error.
# PIN the core version. The firmware is written for the 2.x core API (e.g. the
# HardwareSerial name(RX, TX) pin constructor); core 3.0.0 is a breaking major that
# removes it and fails the build. Leaving this unpinned means a plain image rebuild
# silently jumps to whatever is newest and breaks compilation, so keep it fixed to the
# last known-good 2.x and only bump it deliberately after a test build of both boards.
RUN STM32_URL="https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json" && \
    arduino-cli config init && \
    arduino-cli core update-index --additional-urls "$STM32_URL" && \
    arduino-cli core install STMicroelectronics:stm32@2.10.1 --additional-urls "$STM32_URL" && \
    rm -rf /home/builder/.arduino15/staging \
           /home/builder/.arduino15/tmp/* \
           /home/builder/.arduino15/packages/STMicroelectronics/tools/STM32_SVD
# Cleanup in the same layer: drop the downloaded core archives (staging), any
# extraction leftovers (tmp), and the SVD files (debug-only, not used to compile).
# Saves ~1 GB in the final image. NOTE: this does NOT reduce the transient space the
# core install needs while extracting - the build host still needs a few GB free.

EXPOSE 5000
CMD ["python3", "-u", "app.py"]
