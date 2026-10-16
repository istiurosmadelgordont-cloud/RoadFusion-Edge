# 一笔未完成的交易 · 视频下载

本分支仅保存原创短片的下载文件和还原脚本。

完整时长：8分12秒。形式：电影感原画运镜、中文字幕、原创器乐配乐，无朗读配音。人物与情节为虚构；未来部分为艺术想象。本版为动态分镜形式。

## 直接下载

[打开视频附件下载页](https://github.com/istiurosmadelgordont-cloud/RoadFusion-Edge/releases/tag/silk-road-video-20261017-8421cc96)

附件生成完成后，点击 Assets 中的 `video-compatible.zip`。仓库为私有，需要先登录有权限的 GitHub 账号。

## 通过 Git 拉取

```bash
git clone --depth 1 --single-branch --branch video-download/silk-road-20261017 https://github.com/istiurosmadelgordont-cloud/RoadFusion-Edge.git silk-road-video
cd silk-road-video
python3 restore_video.py
```

Windows 安装了 Python 3 时，也可以双击 `restore-video.bat`。

脚本将30个分卷还原为 `video-compatible.zip`，核对 SHA-256 后，将完整 MP4 解压到 `video/silk-road-compatible.mp4`。

压缩包大小：44,869,261字节。

SHA-256：`8421cc96ae832c6e41e770ddfbab276d8547b1159acf4cc92eeebc367692a15f`。

`play-test.mp4` 为10秒播放测试片段。主项目开发分支为 `main`，此下载分支独立使用。
