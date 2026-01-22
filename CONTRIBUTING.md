# 🛠️ 开发协作指南 (Development Guide)

## 1. 分支规则 (Branch Rules)
* **`master`**：🔴 **只读**。生产环境代码，严禁直接 Push。
* **`develop`**：🟢 **工作分支**。所有开发都在这里进行。

---

## 2. 开发者操作指令 (Developer Workflow)

### 📌 第一次加入项目
```bash
# 克隆项目
git clone https://gitee.com/bitrm2020cv/2025-sentry-navi

# 进入目录
cd 2025-sentry-navi

# ⚠️ 切换到 develop 分支 (严禁在 master 修改)
git checkout develop
``` 
### 日常开发流程
```bash
# 1. 开工前必做：拉取最新代码
git pull origin develop

# ... (写代码) ...

# 2. 提交代码到本地
git add .
git commit -m "feat: 描述具体修改"

# 3. 推送到远程 develop
git push origin develop
# ❌ 禁止操作：
git push origin master
```

## 3. 代码提交规范 (Commit Message Guide)

请在提交代码时，使用以下前缀来描述你的修改类型，格式：`类型: 描述`

* `feat: ...`   ✨ 新增功能 (Feature)
* `fix: ...`    🐛 修复 Bug
* `docs: ...`   📚 仅修改了文档 (Documentation)
* `style: ...`  🎨 调整代码格式，不影响逻辑 (空格、分号等)
* `refactor: ...` ♻️ 代码重构，既没修 Bug 也没加功能
* `perf: ...`   ⚡️ 性能优化
* `test: ...`   ✅ 增加或修改测试代码


**示例：**
> git commit -m "feat: 增加了用户登录界面的UI"
> git commit -m "fix: 修复了数据解析时的空指针崩溃"