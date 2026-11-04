#!/usr/bin/env python3
"""钉钉机器人发送测试 —— 填好 dingtalk.env 后直接跑：python3 test_dingtalk.py"""
import base64, hashlib, hmac, json, os, sys, time, urllib.parse, urllib.request

ENV_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dingtalk.env")

def load_env():
    vals = {}
    if not os.path.isfile(ENV_FILE):
        print(f"❌ 找不到 {ENV_FILE}")
        sys.exit(1)
    with open(ENV_FILE) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            k, v = line.split("=", 1)
            vals[k.strip()] = v.strip()
    return vals

def send_markdown(title, text, token, secret):
    ts = str(int(time.time() * 1000))
    sign_str = f"{ts}\n{secret}"
    sign = urllib.parse.quote_plus(base64.b64encode(
        hmac.new(secret.encode(), sign_str.encode(), hashlib.sha256).digest()))
    url = f"https://oapi.dingtalk.com/robot/send?access_token={token}&timestamp={ts}&sign={sign}"
    body = {"msgtype": "markdown", "markdown": {"title": title, "text": text}}
    req = urllib.request.Request(url, data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as resp:
        return json.loads(resp.read().decode())

if __name__ == "__main__":
    env = load_env()
    token = env.get("DINGTALK_ACCESS_TOKEN", "")
    secret = env.get("DINGTALK_SECRET", "")
    if not token or not secret:
        print(f"❌ 请先在 {ENV_FILE} 里填好 DINGTALK_ACCESS_TOKEN 和 DINGTALK_SECRET")
        sys.exit(1)
    print(f"🔑 token = {token[:6]}...{token[-4:]}")
    print(f"🔑 secret = {secret[:6]}...{secret[-4:]}")
    print("📤 发送测试消息...")
    try:
        result = send_markdown(
            "🧪 消防巡检钉钉测试",
            "## 🧪 消防巡检钉钉测试\n\n"
            f"- **时间**：{time.strftime('%Y-%m-%d %H:%M:%S')}\n"
            f"- **来源**：机器人主机 `/home/nvidia`\n"
            f"- **状态**：✅ 钉钉推送链路正常",
            token, secret,
        )
        if result.get("errcode") == 0:
            print("✅ 发送成功！去钉钉群看看有没有收到测试消息")
        else:
            print(f"❌ 发送失败：{result}")
    except urllib.error.HTTPError as e:
        print(f"❌ HTTP 错误 {e.code}: {e.read().decode()}")
    except Exception as e:
        print(f"❌ 发送异常：{e}")
