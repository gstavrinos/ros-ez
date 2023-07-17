#!/bin/bash
volumes=(
 $1-volume
 $1-volume-bin
 $1-volume-etc
 $1-volume-home
 $1-volume-lib
 $1-volume-lib64
 $1-volume-mnt
 $1-volume-opt
 $1-volume-root
 $1-volume-run
 $1-volume-sbin
 $1-volume-srv
 $1-volume-sys
 $1-volume-usr
 $1-volume-var
)
for volume in "${volumes[@]}"; do
    docker volume rm $volume
done
image=$(echo "$1" | sed 's/ez/_ez/')
docker image rm $image:latest
docker image rm rocker:os_detect_${image}_latest
docker system prune -a
