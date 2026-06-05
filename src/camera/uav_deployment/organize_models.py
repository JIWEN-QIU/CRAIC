#!/usr/bin/env python3
"""
模型组织脚本：从训练输出目录复制导出的 OpenVINO 模型到部署文件夹
"""
import shutil
import argparse
from pathlib import Path
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def organize_models(workspace_root, output_base="uav_deployment"):
    """
    从训练输出目录复制模型到部署目录
    
    Args:
        workspace_root: 项目根目录
        output_base: 输出部署文件夹名称
    """
    workspace_root = Path(workspace_root)
    output_base_path = workspace_root / output_base
    output_base_path.mkdir(parents=True, exist_ok=True)
    
    models_dir = output_base_path / "models"
    models_dir.mkdir(parents=True, exist_ok=True)
    
    # 检测器
    det_src = workspace_root / "runs/detect/runs/cifar100_yolo/cifar_target_detector_ft-2/weights/best_openvino_model"
    det_dst = models_dir / "target_detector_openvino"
    
    # 分类器
    cls_src = workspace_root / "runs/classify/runs/cifar100_yolo/yolov8n_cls_cifar100/weights/best_openvino_model"
    cls_dst = models_dir / "cifar100_classifier_openvino"
    
    logger.info("Organizing models...")
    
    # 复制检测器
    if det_src.exists():
        logger.info(f"Copying detector from {det_src}")
        if det_dst.exists():
            shutil.rmtree(det_dst)
        shutil.copytree(det_src, det_dst)
        logger.info(f"✓ Detector copied to {det_dst}")
    else:
        logger.warning(f"✗ Detector source not found: {det_src}")
    
    # 复制分类器
    if cls_src.exists():
        logger.info(f"Copying classifier from {cls_src}")
        if cls_dst.exists():
            shutil.rmtree(cls_dst)
        shutil.copytree(cls_src, cls_dst)
        logger.info(f"✓ Classifier copied to {cls_dst}")
    else:
        logger.warning(f"✗ Classifier source not found: {cls_src}")
    
    # 显示最终结构
    logger.info("\n=== Final Structure ===")
    for item in models_dir.rglob("*"):
        if item.is_file():
            size = item.stat().st_size / (1024*1024)  # MB
            logger.info(f"{item.relative_to(workspace_root):<80} {size:>8.1f} MB")

def create_symlinks(uav_models_dir, det_dir, cls_dir):
    """
    在无人机上创建符号链接（可选）
    
    Args:
        uav_models_dir: ~/uav_models 目录
        det_dir: 检测器 OpenVINO 目录
        cls_dir: 分类器 OpenVINO 目录
    """
    import os
    uav_models_dir = Path(uav_models_dir)
    uav_models_dir.mkdir(parents=True, exist_ok=True)
    
    # 创建符号链接
    det_link = uav_models_dir / "target_detector_openvino"
    cls_link = uav_models_dir / "cifar100_classifier_openvino"
    
    try:
        if det_link.exists() or det_link.is_symlink():
            det_link.unlink()
        os.symlink(det_dir, det_link)
        logger.info(f"✓ Created symlink: {det_link} -> {det_dir}")
    except Exception as e:
        logger.error(f"✗ Failed to create detector symlink: {e}")
    
    try:
        if cls_link.exists() or cls_link.is_symlink():
            cls_link.unlink()
        os.symlink(cls_dir, cls_link)
        logger.info(f"✓ Created symlink: {cls_link} -> {cls_dir}")
    except Exception as e:
        logger.error(f"✗ Failed to create classifier symlink: {e}")

def main():
    parser = argparse.ArgumentParser(description="Organize models for UAV deployment")
    parser.add_argument('--workspace', default='.', help='Project workspace root')
    parser.add_argument('--output', default='uav_deployment', help='Output deployment folder name')
    
    args = parser.parse_args()
    
    organize_models(args.workspace, args.output)
    
    logger.info("\n✓ Models organized successfully!")
    logger.info(f"Ready to transfer to UAV: {Path(args.workspace) / args.output}")

if __name__ == '__main__':
    main()
