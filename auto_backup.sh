#!/bin/bash
# GitHub 自动备份脚本
# 每周自动推送代码到 GitHub

# 进入你的代码目录
cd ~/桌面/vision01 || exit

# 拉取远程最新代码（避免冲突）
git pull origin master

# 添加所有变更
git add .

# 提交变更（如果没有变更则跳过）
git commit -m "Auto backup: $(date +'%Y-%m-%d %H:%M:%S')" || echo "No changes to commit"

# 推送到 GitHub
git push origin master
