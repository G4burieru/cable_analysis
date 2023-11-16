# This Dockerfile is a documentation example and might not build after a copy/paste process

#arm64v8 is the pre-requisite for running the container on the VPU.
FROM arm64v8/python:3.9.6-slim-buster

#Security updates
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get -y update && apt-get -y upgrade
RUN apt-get -y install vim
RUN apt-get update && apt-get install ffmpeg libsm6 libxext6  -y

#Create and activate virtual environment. This is not needed right now, but useful for multistage builds.
RUN python -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Your normal pip installation, within the venv. We also update pip.
RUN pip install -U pip && pip install numpy ifm3dpy opencv-contrib-python opencv-python

#For security reasons, using a "user" is recommended
RUN useradd --create-home pythonuser
USER pythonuser
RUN mkdir /home/pythonuser/images
COPY saveimgs.py /home/pythonuser

#Easier to debug the container if issues are happening
ENV PYTHONFAULTHANDLER=1
