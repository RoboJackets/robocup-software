# syntax=docker/dockerfile:1

# BuildKit automatically sets this to amd64 or arm64.
ARG TARGETARCH
FROM robojackets/robocup-software-dev:${TARGETARCH}

# Display Qt applications in the existing noVNC desktop.
ENV DISPLAY=:1 \
    QT_X11_NO_MITSHM=1