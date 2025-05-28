import os
import zipfile
import json
from pathlib import Path

def load_config():
    """加载配置文件"""
    config_path = 'packaging_config.json'
    if os.path.exists(config_path):
        with open(config_path, 'r', encoding='utf-8') as f:
            return json.load(f)
    else:
        # 默认配置
        default_config = {
            "include_folders": ["libraries", "middlewares", "project"],
            "exclude_folders": [".git", "__pycache__"],
            "exclude_files": [".gitignore"],
            "exclude_extensions": [".pyc"]
        }
        with open(config_path, 'w', encoding='utf-8') as f:
            json.dump(default_config, f, indent=4)
        return default_config

def should_exclude(file_path, config):
    """检查文件是否应该被排除"""
    # 检查是否在排除文件夹中
    for folder in config['exclude_folders']:
        if folder in file_path.parts:
            return True
    
    # 检查是否在排除文件中
    if file_path.name in config['exclude_files']:
        return True
    
    # 检查扩展名
    if file_path.suffix.lower() in config['exclude_extensions']:
        return True
    
    return False

def package_project():
    """打包项目"""
    config = load_config()
    output_filename = "AT32F421C8T7_LLC_Buck_Package.zip"
    
    print("打包配置:")
    print(f"包含的文件夹: {', '.join(config['include_folders'])}")
    print(f"排除的文件夹: {', '.join(config['exclude_folders'])}")
    print(f"排除的文件: {', '.join(config['exclude_files'])}")
    print(f"排除的扩展名: {', '.join(config['exclude_extensions'])}")
    print("\n开始打包...")
    
    with zipfile.ZipFile(output_filename, 'w', zipfile.ZIP_DEFLATED) as zipf:
        for folder in config['include_folders']:
            if not os.path.exists(folder):
                print(f"警告: 文件夹 {folder} 不存在，跳过")
                continue
                
            for root, dirs, files in os.walk(folder):
                # 过滤排除的文件夹
                dirs[:] = [d for d in dirs if d not in config['exclude_folders']]
                
                for file in files:
                    file_path = Path(root) / file
                    if not should_exclude(file_path, config):
                        arcname = os.path.relpath(file_path, start='.')
                        zipf.write(file_path, arcname)
                        print(f"已添加: {arcname}")
    
    print(f"\n打包完成! 输出文件: {output_filename}")

if __name__ == "__main__":
    package_project()
