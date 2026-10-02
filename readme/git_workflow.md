# Git / GitHub 两机协作速查

**场景：** 一台电脑写代码，另一台电脑调试（如实验室机器），通过 Git + GitHub 同步。


## 1. 一次性配置（每台电脑都要做一遍）

```bash
git config --global user.name "queen1166"
git config --global user.email "3365182535@qq.com"
```


**生成 SSH 密钥**（每台电脑一把，不共用）：

```bash
ssh-keygen -t ed25519 -C "3365182535@qq.com"
```

> 生成一对密钥：`github_ed25519`（私钥，留在本机，**永远不要给别人**）、`github_ed25519.pub`（公钥，给 GitHub）。

把 **`.pub` 文件的内容**复制到 GitHub → Settings → SSH and GPG keys → New SSH key。

**验证：**

```bash
ssh -T git@github.com
```

> 回 `Hi xxx! You've successfully authenticated` 就是通了。



## 2. 在电脑 A 上：把代码推上去

```bash
git init                        # 把这个目录变成 git 仓库（生成隐藏的 .git）
git add .                       # 把所有改动放进暂存区（准备提交）
git commit -m "完成排序函数"      # 提交到本地仓库，"" 里是这次改了什么
git branch -M main              # 把默认分支改名成 main
git remote add origin git@github.com:queen1166/仓库名.git   # 记住云端地址，起名叫 origin
git push -u origin main         # 推上去；-u 是记住关联，以后直接 git push 就行
```

> `origin` 只是给远程地址起的**别名**，你可以叫别的，但全世界的教程都用 `origin`，跟着用就好。

以后每次改完：

```bash
git add .
git commit -m "修好了越界 bug"
git push
```

---

## 3. 在电脑 B 上：把代码拿下来

**第一次**（仓库还不存在）：

```bash
git clone git@github.com:queen1166/仓库名.git
```

> 一条命令干三件事：下载全部代码 + 建立本地仓库 + 自动配好 `origin`。

**以后每次**（仓库已存在，只要最新代码）：

```bash
git pull
```

> `pull` = `fetch`（下载）+ `merge`（合并到当前分支）。**动手写代码前先 pull，这是铁律。**

---

## 主办方机器专项


**1. 装 Git 并配身份**

```bash
git config --global user.name "queen1166"
git config --global user.email "3365182535@qq.com"
```

**2. 生成那台机器自己的密钥**

```bash
ssh-keygen -t ed25519 -C "3365182535@qq.com"     # 一路回车即可
cat ~/.ssh/id_ed25519.pub                        # 输出的一行，贴到 GitHub
```

> ⚠️ **不要去拷自己电脑上的私钥。** 一台机器一把钥匙；两把公钥可以挂在同一个 GitHub 账号下，互不干扰。共用私钥的话，任何一台出问题都得全部重配。

**3. 把公钥加到 GitHub**

https://github.com/settings/keys → New SSH key → Title 起名 `主办方机器`（方便日后区分是哪台的）

**4. 验证并拉代码**

```bash
ssh -T git@github.com      # 期望输出：Hi queen1166! You've successfully authenticated
git clone git@github.com:queen1166/autoaim.git
```

### 每天在两台机器上的固定动作

```bash
# 开工前（两台都要先拉）—— 铁律
git pull

# 收工前
git add .
git commit -m "说明"
git push
```

> 主办方机器上调试完的改动，**记得 push 再走人**。否则回到自己电脑，那部分改动就断在那台机器上了。



## 4. 日常命令表

| 命令 | 含义 | 什么时候用 |
| --- | --- | --- |
| `git status` | 看现在有哪些文件改了 / 没提交 | **最常用**，迷茫时先敲它 |
| `git add <文件>` | 把某个文件的改动放进暂存区 | 想分开提交时 |
| `git add .` | 全部放进暂存区 | 图省事 |
| `git commit -m "说明"` | 提交到本地 | 一个完整的小改动做完时 |
| `git push` | 推到 GitHub | 想让另一台电脑看到时 |
| `git pull` | 从 GitHub 拉最新 | **每次开工前** |
| `git log --oneline --graph` | 看提交历史 | 想看清发生了什么 |
| `git diff` | 看还没 add 的具体改动 | 提交前检查自己改了什么 |
| `git branch` | 列出本地分支，前面带 `*` 的是当前所在分支 | 不确定自己在哪条分支时 |
| `git branch -a` | 列出所有分支，含云端的（`remotes/origin/...`） | 想看云端有哪些分支 |
| `git switch -c <分支名>` | 新建一条分支并切过去 | 想试新东西、又不想弄脏 main |
| `git switch <分支名>` | 切换到已有的分支 | 切回 main，或切到别的实验分支 |
| `git merge <分支名>` | 把指定分支合并进**当前**分支 | 实验成功，想并回 main |
| `git push -u origin <分支名>` | 把新分支推到 GitHub | 让另一台电脑也能拿到这条分支 |
| `git branch -d <分支名>` | 删除分支（已合并的才删得掉） | 分支用完清理 |

> **在你这个两机场景里怎么用：** 在主办方机器上做试验性调试时，先 `git switch -c 调试-0820` 开一条分支再动手，就不会污染 main。试成了切回 main 合并；试砸了直接切回 main 把分支扔掉，main 始终干净。
>
> ⚠️ 切换分支前先 `git commit`（或 `git stash` 暂存）。有未提交的改动时 git 可能拦着你不让切 —— 它怕把你的改动弄丢。

---