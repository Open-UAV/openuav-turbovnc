# OpenUAV Flight simulator for single and multi UAVs

https://openuas.us

### Introduction
  The OpenUAV is a simulation environment developed for Unmanned Aerial Vehicles. 
  The goal of this framework is to have an easy to setup, UAV testing framework for single and multi-UAV simulations.
  The containerized architecture makes this framework capable of scaling and hosting mutiple simulations in an
  on-premise or cloud environment. This project is a modification of the [willkessler/nvidia-docker-novnc](https://github.com/willkessler/nvidia-docker-novnc) to include PX4/QGroundControl software packages UAV simulations.

To create a simulation session, please create an account at CPS-VO [https://cps-vo.org/group/OpenUAV](https://cps-vo.org/group/OpenUAV). If you are using it for educational use-case, mention the reason while creating an account. For more details, [https://openuav.us/#cpsvo](https://openuav.us/#cpsvo)

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


## Building a plain vanilla Google Compute GPU host

1. Spin up a GC GPU host on the google console.  Make sure it has at least one Tesla K80 GPU, and decent amount of VCPUs (e.g. 4, and enough disk space, at least 50Gb). Zone `us-east-1c` seems to be the best choice as of April 1, 2018.
2. Upload this repo and unpack it in `/root/build` or wherever you like as a temporary location.
3. Run `preinstall.sh`. This just runs `apt-get update` and puts in `screen` and `emacs` for getting started.
4. Run `build.sh`. This will build everything needed to start up a nvidia-docker2 container with Ubuntu 18.04 and Lubuntu desktop.

## Building on a local machine with GPU

1. Run `nvidia-xconfig --query-gpu-info` and modify the `autonomous_sys_build/xorg.conf` the GPU BusId.
2. Run `preinstall.sh`. This runs `apt-get update` and puts in `screen` and `emacs` for getting started.
3. Run `build.sh`. This will build everything needed to start up a nvidia-docker2 container with Ubuntu 18.04 and Lubuntu desktop.

### Running the container

To run the container on this host, use `run.sh`. Note that NoVNC will
expect connections on port 40001. Then surf to your host on that port.


### Nginx configuration
Setting up OpenUAV

```
server {
        server_name ~^term-(?<subnum>\d+)\.openuav\.us$;
        listen 443 ssl; # managed by Certbot
        ssl_certificate /etc/letsencrypt/live/openuav.us/fullchain.pem; # managed by Certbot
        ssl_certificate_key /etc/letsencrypt/live/openuav.us/privkey.pem; # managed by Certbot
        include /etc/letsencrypt/options-ssl-nginx.conf; # managed by Certbot
        ssl_dhparam /etc/letsencrypt/ssl-dhparams.pem; # managed by Certbot
        auth_basic "Private Property";
        auth_basic_user_file /etc/nginx/.htpasswd;
        location / {
                proxy_set_header Upgrade $http_upgrade;
                proxy_set_header Connection $connection_upgrade;
                proxy_set_header X-Real-IP $remote_addr;
                proxy_set_header Host $host;
                proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
                if ($subnum ~ [0-9])
                {
                        proxy_pass http://127.0.0.1:400$subnum;
                }
                if ($subnum ~ [1-9][0-9])
                {
                        proxy_pass http://127.0.0.1:40$subnum;
                }
                proxy_ssl_certificate /etc/letsencrypt/live/openuav.us/fullchain.pem; # managed by Certbot
                proxy_ssl_certificate_key /etc/letsencrypt/live/openuav.us/privkey.pem; # managed by Certbot
                proxy_hide_header 'x-frame-options';
                proxy_read_timeout 61s;

                # Disable cache
                proxy_buffering off;
        }
}
```


### OpenUAV video & paper

Video https://www.youtube.com/watch?v=Tcz61dXIyAs
Paper https://arxiv.org/abs/1910.00739

### Known issues

1. In the local setup, if you switch users the containers have to be restarted to update for Xorg changes.
   A fix for it is if your machine has more than one GPU, then you can run containers on the other GPU by running an xserver in it.
   
### DockerHub Image

1. You can try our https://hub.docker.com/r/dreamslab/openuav image to setup the system (GCP K80 GPU), it's recommended to use `build.sh`.
