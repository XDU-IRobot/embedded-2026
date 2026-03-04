# librm-docs

librm 的文档网站，使用[Docusaurus](https://docusaurus.io/)编写。

### 开发指南

首先用 Git LFS 拉取文档中的所有大文件（图片、动图、视频等）：

```bash
git lfs pull
```

然后安装依赖并启动开发服务器即可：

```bash
npm install       ### 安装依赖，如果使用其他包管理器，更换npm为yarn或pnpm
npm run build     ### 构建静态文件
npm start         ### 启动开发服务器
```
