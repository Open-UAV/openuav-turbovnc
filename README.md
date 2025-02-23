# OpenUAV Cloud Testbed: a Collaborative Design Studio for Field Robotics

https://openuas.us

### Introduction
  The OpenUAV is a simulation environment developed for Unmanned Aerial Vehicles. 
  The goal of this framework is to have an easy to setup, UAV testing framework for single and multi-UAV simulations.
  The containerized architecture makes this framework capable of scaling and hosting mutiple simulations in an
  on-premise or cloud environment. This project is a modification of the [willkessler/nvidia-docker-novnc](https://github.com/willkessler/nvidia-docker-novnc) to include PX4/QGroundControl software packages UAV simulations.

To create a simulation session, please create an account at Cyber-Physical Systems Virtual Organization (CPS-VO) [https://cps-vo.org/group/OpenUAV](https://cps-vo.org/group/OpenUAV). If you are using it for educational use-cases, please mention the reason and university while creating an account. For more details, [https://openuav.us/#cpsvo](https://openuav.us/#cpsvo)

### Acknowledgements:
- NSF grant CNS-1521617
- USDA grant 2015-67021-23857
- GRASP Lab, University of Pennsylvania
- Penn Aerial Robotics
- School of Earth and Space Exploration, ASU
- Arizona State University
- [@willkessler] (https://github.com/willkessler)

**Authors:**
   **[Harish Anand](https://web.asu.edu/jdas/people/harish-anand), hanand4 (at) asu (dot) edu;**
   
   **[Prof. Jnaneshwar "JD" Das](https://sese.asu.edu/node/3438 "Jnaneshwar Das"), [Distributed Robotic Exploration and Mapping Systems Laboratory](https://web.asu.edu/jdas), ASU School of Earth and Space Exploration**
 
### UPDATE 
- Support for CUDA inside containers (useful for YOLO and other object detection algorithms)
- Unity3D installation instructions are provided here. [https://openuas.us/#unity](https://openuas.us/#unity)

## System Requirements and Versions

### Current Software Versions
- Base OS: Ubuntu 22.04 LTS
- CUDA: 12.3.0
- ROS2: Humble
- Python: 3.10
- GPU Support: NVIDIA GPUs with CUDA capability

### Container Images
After running `build.sh`, three Docker images are created:
- openuav:base-cuda-12.3.0-ubuntu22.04
- openuav:ros-cuda-12.3.0-ubuntu22.04 
- openuav:px4-cuda-12.3.0-ubuntu22.04

## Building Instructions

### Prerequisites
- NVIDIA GPU with updated drivers
- Docker with NVIDIA container toolkit
- Ubuntu 22.04 (recommended) or compatible Linux distribution

### Building Steps
1. Clone the repository:
   ```bash
   git clone https://github.com/Open-UAV/openuav-turbovnc.git
   cd openuav-turbovnc
   ```

2. For local machine setup with GPU:
   - Run `nvidia-smi` to verify GPU access
   - Run `nvidia-xconfig --query-gpu-info` and update `autonomous_sys_build/xorg.conf` with your GPU's BusId
   - Execute `./preinstall.sh`
   - Run `./build.sh`

3. For cloud setup (e.g., Google Cloud with GPU):
   - Follow the existing instructions under "Building a plain vanilla Google Compute GPU host"
   - Ensure the instance has NVIDIA Tesla GPU support
   - Execute the build scripts as described above

## Building a plain vanilla Google Compute GPU host

1. Spin up a GC GPU host on the google console. Make sure it has at least one Tesla K80 GPU, and decent amount of VCPUs (e.g. 4, and enough disk space, at least 50Gb). Zone `us-east-1c` seems to be the best choice as of April 1, 2018.
2. Upload this repo and unpack it in `/root/build` or wherever you like as a temporary location.
3. Run `preinstall.sh`. This just runs `apt-get update` and puts in `screen` and `emacs` for getting started.
4. Run `build.sh`. This will build everything needed to start up a nvidia-docker2 container with Ubuntu 18.04 and Lubuntu desktop.

## Building on a local machine with GPU

1. Run `nvidia-xconfig --query-gpu-info` and modify the `autonomous_sys_build/xorg.conf` with your GPU's BusId and name.
2. Run `preinstall.sh`. This runs `apt-get update` and puts in `screen` and `emacs` for getting started.
3. Run `build.sh`. This will build everything needed to start up a nvidia-docker2 container with Ubuntu 18.04 and Lubuntu desktop.

### Running the container

To run the container on this host, use `run.sh`. Note that NoVNC will
expect connections on port 40001. Then surf to your host on that port.

### Further details

1. There are 3 docker images created after running `build.sh`, 
   - openuav:base-cuda-10.2-ubuntu18.04 
   - openuav:ros-cuda-10.2-ubuntu18.04
   - openuav:px4-cuda-10.2-ubuntu18.04

  The first image contains a basic linux ubuntu 18.04 desktop with chrome and pycharm installed. These session are used for python programming courses and can avoid any of the GPU requirements (change the base image in Dockerfile.base to `nvidia/opengl:1.0-glvnd-runtime`). Second image is a robotics specific image that contains ros melodic and gazebo 9.0.0 installed. The third image installs all the necessary PX4 and QGroundControl software to do flight simulations. 

2. To access, containers through the sub-domain based url, you need to setup a dns proxy to obtain the docker internal DNS for routing. This can be achieved by creating a reverse proxy NginX container, that serves the internal DNS to host machine. 
   - Create a docker network for openuav containers, `docker network create -d bridge cpsvo`.
   - Create a docker NginX container. Command is `docker run --name dns-proxy --network=cpsvo -d nginx`.
   - Enter the nginx container, install vim/nano (for editing nginx.conf file) and net-tools. (`apt update && apt install vim net-tools`)
   - Replace the NginX containers `/etc/nginx/nginx.conf` with the following conf. This provides docker DNS to host machine (127.0.0.11 from inside docker) 
      ```
      user  nginx;
      worker_processes  1;

      error_log  /var/log/nginx/error.log warn;
      pid        /var/run/nginx.pid;


      events {
          worker_connections  1024;
      }


      http {
          include       /etc/nginx/mime.types;
          default_type  application/octet-stream;

          log_format  main  '$remote_addr - $remote_user [$time_local] "$request" '
                            '$status $body_bytes_sent "$http_referer" '
                            '"$http_user_agent" "$http_x_forwarded_for"';

          access_log  /var/log/nginx/access.log  main;

          sendfile        on;
          #tcp_nopush     on;

          keepalive_timeout  65;

          #gzip  on;

          include /etc/nginx/conf.d/*.conf;
      }
      stream {
         upstream ssh_openuav {
             server 127.0.0.11:53;
         }
         server {
             listen        53 udp;
             proxy_pass    ssh_openuav;

         }
      }
      ```
   - Install NginX on the host machine and modify the `/etc/nginx/sites-enabled/default` file to include the following server configuration. 
     You should replace `\.openuav\.us$;` with your domain name and the cpsvo in the beginning can be replaced with a any other name. You must 
     also replace 172.18.0.2 in `resolver 172.18.0.2 valid=2s;` to the IP address of the NginX container (`docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' dns-proxy`). You must also replace the ssl_certificate and ssl_certificate_key variables with your variables.
       ```
      server {
              server_name ~^cpsvo-(?<uuid>\S+)\.openuav\.us$;
              listen 443 ssl; # managed by Certbot
              ssl_certificate /etc/letsencrypt/live/openuav.us-0001/fullchain.pem; # managed by Certbot
              ssl_certificate_key /etc/letsencrypt/live/openuav.us-0001/privkey.pem; # managed by Certbot
              resolver 172.18.0.2 valid=2s; 
              location / {
                      proxy_set_header Upgrade $http_upgrade;
                            proxy_set_header Connection $connection_upgrade;
                            proxy_set_header X-Real-IP $remote_addr;
                            proxy_set_header Host $host;
                            proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
                            proxy_pass http://cpsvo-$uuid:40001;
                            proxy_hide_header 'x-frame-options';
                            proxy_read_timeout 61s;
                            add_header Cache-Control "no-store";
                            # Disable cache
                            proxy_buffering off;
              }
      }

      ```

### OpenUAV paper

Paper https://arxiv.org/abs/1910.00739

### Known issues

1. In the local setup, if you switch users in the local machine the containers have to be restarted to update for Xorg changes.
   A fix for this is to run the xserver on a GPU specifically used for OpenUAV work. (for more details https://openuav.us/#onpremise )   
   ```
   /usr/lib/xorg/Xorg -core :1 -seat seat1 -auth /var/run/lightdm/root/:1 -nolisten tcp vt8 -novtswitch -config /etc/X11/xorg.conf.openuav
   ```
2. Working with AMD GPUs (https://github.com/Open-UAV/openuav-turbovnc/issues/29)
3. GLX issues

    Applications like Gazebo need libX11.so and libGLX.so to work. For all applications inside the container, X11 is provided by the TurboVNC's Xvnc (acting both as X window server and as a vnc server). VirtualGL is used to redirect applications (like Gazebo or Unity) that make GLX calls (3D rendering) to run the graphics rendering in a dedicated GPU and we achieve this by preloading each command you run in the terminal through vglrun command. Thus VirtualGL takes care of libGLX.so.

    Regarding env variables, the `DISPLAY` env variable points to `:1.0` which is the TurboVNC X window session (`Xvnc`), and for applications that needs 3D acceleration, the application does GLX calls which are handled by an Xserver (with GPU) running at `:0.0`. We provide details of second X server using the variable `VGL_DISPLAY` which is set to :0.0 on all containers. If `VGL_DISPLAY` is not set on the container, it assumes the value `:0.0`.

    You can check for any GLX issues, by using the command `glxgears`. Glxgear application's 2D window will be handled by TurboVNC (Xvnc) and 3D rendering will be handled by an X with GPU (virtualGL should redirect this correctly).

    If you are facing GLX issues, try checking X server running at (`:0.0`) with GPU support is connected properly. You can pass a host X server with GPU ti the inside of the container as `/tmp/X11-unix/X0`. This connection is usually the issue for most GLX problems.


### DockerHub Image

1. You can try our https://hub.docker.com/r/dreamslab/openuav image to setup the system (GCP K80 GPU), it's recommended to use `build.sh`.

### Container Architecture Updates
The current architecture uses TurboVNC for remote desktop access and supports:
- Modern CUDA applications and GPU acceleration
- ROS2 Humble with Gazebo integration
- Updated development tools including VS Code
- Improved GPU passthrough for 3D applications

### Version History
- 2024: Updated to Ubuntu 22.04, CUDA 12.3.0, ROS2 Humble
- Previous: Ubuntu 18.04, CUDA 10.2, ROS Melodic
