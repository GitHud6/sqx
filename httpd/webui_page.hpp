#pragma once

namespace laserctl::httpd {

// Small HTML entry point. Styles and behaviour are delivered as sequential
// static assets so an HTTP refresh never needs to send the entire portal at once.
constexpr char kWebUiPage[] = R"HTML(<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <meta name="color-scheme" content="dark">
  <title>Laser Controller 控制台</title>
  <link rel="stylesheet" href="/app.css">
  <script>document.querySelector('link[href="/app.css"]').addEventListener('load', () => { const script = document.createElement('script'); script.src = '/app.js'; document.head.appendChild(script); }, { once: true });</script>
</head>
<body>
  <header>
    <div><h1>Laser Controller 控制台</h1><div class="subtle">本地设备入口 · 初始界面</div></div>
    <div class="connection-group">
      <div id="position" class="badge offline">位置待确认</div>
      <div id="connection" class="badge offline">WebSocket 连接中</div>
    </div>
  </header>

  <section>
    <div class="toolbar">
      <h2>机器准备</h2>
      <button id="home" type="button">回零（$H）</button>
    </div>
    <div id="home-notice" class="notice">连接后将读取位置状态；未知位置时必须先回零。</div>
  </section>

  <section>
    <h2>命令</h2>
    <form id="command-form">
      <input id="command" autocomplete="off" spellcheck="false" placeholder="例如：$I、?、G0 X10 F1000" aria-label="设备命令">
      <button id="send" type="submit" disabled>发送</button>
    </form>
    <p class="subtle">命令会通过 WebSocket 发送到现有协议层；页面不直接调用运动模块。</p>
  </section>

  <section>
    <h2>设备消息</h2>
    <pre id="messages" aria-live="polite">正在连接设备…</pre>
  </section>

  <section>
    <div class="toolbar">
      <h2>无线网络</h2>
      <button id="wifi-read" type="button" disabled>读取配置</button>
    </div>
    <form id="wifi-form" class="wifi-form">
      <label for="wifi-ssid">STA SSID</label>
      <input id="wifi-ssid" type="text" autocomplete="off" spellcheck="false" aria-label="STA SSID">
      <label for="wifi-password">STA 密码</label>
      <input id="wifi-password" type="text" autocomplete="off" spellcheck="false" aria-label="STA 密码">
      <label for="wifi-mode">连接策略</label>
      <select id="wifi-mode" aria-label="无线连接策略">
        <option value="AP">AP</option>
        <option value="STA">STA</option>
        <option value="AP_STA">AP_STA</option>
      </select>
      <div class="wifi-actions">
        <button id="wifi-apply" type="submit" disabled>保存并切换网络</button>
      </div>
    </form>
    <div id="wifi-notice" class="notice">读取后可修改保存的无线配置。</div>
  </section>

  <section>
    <div class="toolbar">
      <h2>SD 卡文件</h2>
      <button id="refresh-files" type="button">刷新</button>
    </div>
    <form id="upload-form" class="upload">
      <div class="job-progress" aria-live="polite">
        <div class="job-progress-head">
          <span>SD 作业进度（文件供料）</span>
          <span class="job-progress-actions">
            <button id="job-hold" type="button" title="暂停雕刻" disabled>暂停</button>
            <button id="job-resume" type="button" title="继续雕刻" disabled>继续</button>
            <button id="job-reset" class="reset" type="button" title="终止作业并复位" disabled>复位</button>
          </span>
        </div>
        <progress id="job-progress" max="1000" value="0">0%</progress>
        <div id="job-progress-text" class="notice">空闲</div>
      </div>
      <input id="upload-file" type="file" required aria-label="选择要上传的文件">
      <button id="upload" type="submit">上传到根目录</button>
      <div id="upload-progress-panel" class="upload-progress" hidden aria-live="polite">
        <progress id="upload-progress" max="1000" value="0">0%</progress>
        <div id="upload-progress-text" class="notice">等待上传。</div>
      </div>
    </form>
    <div id="file-notice" class="notice">尚未加载文件列表。</div>
    <ul id="file-list" class="file-list"></ul>
  </section>

  <section>
    <h2>当前接口</h2>
    <dl class="api">
      <dt>HTTP</dt><dd><code>GET /</code>、<code>/api/v1/storage/*</code>、<code>/command</code></dd>
      <dt>WebSocket</dt><dd><code>ws://&lt;设备地址&gt;:8849/</code></dd>
      <dt>文件服务</dt><dd><code>GET /api/v1/storage/list</code>、<code>PUT /api/v1/storage/file</code></dd>
    </dl>
  </section>


</body>
</html>
)HTML";

}  // namespace laserctl::httpd
