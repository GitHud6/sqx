#pragma once

namespace ray2::httpd {

// The first-party WebUI deliberately stays a single embedded page. It has no
// runtime storage dependency and gives the device a stable local entry point
// while the API and interaction design evolve.
constexpr char kWebUiPage[] = R"HTML(<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <meta name="color-scheme" content="dark">
  <title>RayCube 控制台</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, sans-serif; background: #10151d; color: #edf2f7; }
    * { box-sizing: border-box; }
    body { max-width: 760px; margin: 0 auto; padding: 24px 16px 40px; }
    header { display: flex; justify-content: space-between; gap: 16px; align-items: center; margin-bottom: 24px; }
    h1 { font-size: 1.35rem; margin: 0; }
    .subtle { color: #9cacbd; font-size: .9rem; }
    .badge { border: 1px solid #566577; border-radius: 999px; padding: 5px 10px; font-size: .82rem; }
    .badge.online { border-color: #48c78e; color: #48c78e; }
    .badge.offline { border-color: #e57373; color: #ff9c9c; }
    .connection-group { display: flex; flex-wrap: wrap; justify-content: end; gap: 8px; }
    section { border: 1px solid #283443; border-radius: 10px; background: #151d28; margin: 14px 0; padding: 16px; }
    h2 { font-size: 1rem; margin: 0 0 10px; }
    code, pre, input, button { font-family: ui-monospace, Consolas, monospace; }
    pre { height: 250px; overflow: auto; margin: 0; padding: 12px; border-radius: 6px; background: #0b1017; color: #c9d7e5; white-space: pre-wrap; word-break: break-word; }
    form { display: flex; gap: 8px; }
    input { flex: 1; min-width: 0; padding: 10px; border: 1px solid #445468; border-radius: 6px; background: #0b1017; color: inherit; }
    button { padding: 10px 14px; border: 1px solid #4e7ba8; border-radius: 6px; background: #1c4f7e; color: white; cursor: pointer; }
    button:disabled { opacity: .5; cursor: default; }
    .toolbar { display: flex; justify-content: space-between; align-items: center; gap: 10px; margin-bottom: 12px; }
    .file-list { list-style: none; padding: 0; margin: 0; border-top: 1px solid #283443; }
    .file-row { display: grid; grid-template-columns: 1fr auto auto; align-items: center; gap: 10px; padding: 10px 2px; border-bottom: 1px solid #283443; }
    .file-name { overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
    .file-size { color: #9cacbd; font-size: .85rem; }
    .file-actions { display: flex; gap: 6px; }
    .file-actions button { padding: 5px 8px; font-size: .8rem; }
    .file-actions .danger { border-color: #8d4e58; background: #6c2833; }
    .file-actions .engrave { border-color: #4b8b72; background: #176044; }
    .notice { min-height: 1.3em; margin: 10px 0 0; color: #9cacbd; font-size: .9rem; }
    .notice.error { color: #ff9c9c; }
    .job-progress { margin: 12px 0 4px; }
    .job-progress-head { display: flex; justify-content: space-between; gap: 12px; color: #9cacbd; font-size: .9rem; }
    .job-progress progress { width: 100%; height: 12px; margin-top: 6px; accent-color: #48b58a; }
    .job-progress-actions { display: flex; gap: 6px; }
    .job-progress-actions button { min-width: 32px; padding: 4px 8px; }
    .job-progress-actions .reset { border-color: #8d4e58; background: #6c2833; }
    .upload { align-items: center; flex-wrap: wrap; }
    .upload input[type=file] { min-width: 220px; }
    .api { display: grid; grid-template-columns: max-content 1fr; gap: 8px 14px; margin: 0; }
    .api dt { color: #8ea6bd; }
    .api dd { margin: 0; }
  </style>
</head>
<body>
  <header>
    <div><h1>RayCube 控制台</h1><div class="subtle">本地设备入口 · 初始界面</div></div>
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

  <script>
    (() => {
      const status = document.getElementById('connection');
      const position = document.getElementById('position');
      const output = document.getElementById('messages');
      const form = document.getElementById('command-form');
      const input = document.getElementById('command');
      const submit = document.getElementById('send');
      const refreshFilesButton = document.getElementById('refresh-files');
      const uploadForm = document.getElementById('upload-form');
      const uploadFile = document.getElementById('upload-file');
      const uploadButton = document.getElementById('upload');
      const fileNotice = document.getElementById('file-notice');
      const fileList = document.getElementById('file-list');
      const homeButton = document.getElementById('home');
      const homeNotice = document.getElementById('home-notice');
      const jobProgress = document.getElementById('job-progress');
      const jobProgressText = document.getElementById('job-progress-text');
      const jobHoldButton = document.getElementById('job-hold');
      const jobResumeButton = document.getElementById('job-resume');
      const jobResetButton = document.getElementById('job-reset');
      let socket;
      let homeReady = false;
      let homingRequested = false;
      let homeDeadline = 0;
      let homePollTimer = 0;
      let progressPollTimer = 0;
      let sdJobActive = false;
      let realtimeMotionState = 'Idle';

      function append(message) {
        output.textContent += `${output.textContent ? '\n' : ''}${message}`;
        output.scrollTop = output.scrollHeight;
      }

      function setConnection(online) {
        status.className = `badge ${online ? 'online' : 'offline'}`;
        status.textContent = online ? 'WebSocket 已连接' : 'WebSocket 未连接';
        submit.disabled = !online;
        updateHomeButton();
        updateEngraveButtons();
      }

      function updateHomeButton() {
        homeButton.disabled = !socket || socket.readyState !== WebSocket.OPEN || homingRequested;
      }

      function updateEngraveButtons() {
        const available = homeReady && !sdJobActive && socket && socket.readyState === WebSocket.OPEN;
        document.querySelectorAll('[data-engrave]').forEach((button) => { button.disabled = !available; });
        updateJobControls();
      }

      function updateJobControls() {
        const connected = socket && socket.readyState === WebSocket.OPEN;
        jobHoldButton.disabled = !connected || !sdJobActive || realtimeMotionState !== 'Run';
        jobResumeButton.disabled = !connected || !sdJobActive || realtimeMotionState !== 'Hold';
        jobResetButton.disabled = !connected || !sdJobActive;
      }

      function stopProgressPolling() {
        window.clearInterval(progressPollTimer);
        progressPollTimer = 0;
      }

      function startProgressPolling() {
        if (progressPollTimer) return;
        sendCommand('?', false);
        progressPollTimer = window.setInterval(() => sendCommand('?', false), 500);
      }

      function beginJobProgressMonitoring() {
        sdJobActive = true;
        jobProgress.value = 0;
        jobProgressText.textContent = 'starting · 0.0% · 第 0 行';
        updateEngraveButtons();
      }

      function updateJobProgress(state, tenths, line) {
        const active = state === 'starting' || state === 'running' || state === 'draining';
        sdJobActive = active;
        jobProgress.value = Math.max(0, Math.min(1000, tenths));
        jobProgressText.textContent = `${state} · ${(tenths / 10).toFixed(1)}% · 第 ${line} 行`;
        updateEngraveButtons();
      }

      function updateJobProgressFromStatus(message) {
        // `SD` is a regular realtime field. Do not require it to remain the
        // final field, so future status additions cannot silently disable the
        // local portal progress display.
        const field = message.match(/(?:^|\|)SD:([^|\r\n>]+)/);
        if (!field) return;
        const values = field[1].split(',');
        if (values.length !== 3) return;
        const percent = Number.parseFloat(values[1]);
        const line = Number.parseInt(values[2], 10);
        if (!Number.isFinite(percent) || !Number.isFinite(line)) return;
        updateJobProgress(values[0], Math.round(percent * 10), line);
      }

      function updateRealtimeMotionState(message) {
        const status = message.match(/^<([^|>]+)/);
        if (!status) return;
        realtimeMotionState = status[1];
        updateJobControls();
      }

      function setHomeReady(ready, message) {
        homeReady = ready;
        position.className = `badge ${ready ? 'online' : 'offline'}`;
        position.textContent = ready ? '位置已回零确认' : '位置待确认';
        homeNotice.textContent = message;
        homeNotice.className = 'notice';
        updateHomeButton();
        updateEngraveButtons();
      }

      function failHoming(message) {
        homingRequested = false;
        window.clearTimeout(homePollTimer);
        setHomeReady(false, message);
        homeNotice.classList.add('error');
      }

      function sendCommand(command, show = true) {
        if (!socket || socket.readyState !== WebSocket.OPEN) return false;
        socket.send(command);
        if (show) append(`> ${command}`);
        return true;
      }

      function requestPositionStatus() {
        sendCommand('$POS/STATUS', false);
      }

      function pollHoming() {
        window.clearTimeout(homePollTimer);
        if (!homingRequested) return;
        if (Date.now() > homeDeadline) {
          failHoming('回零超时；请检查限位开关与机器状态后重试。');
          return;
        }
        sendCommand('$H/STATUS', false);
        requestPositionStatus();
        homePollTimer = window.setTimeout(pollHoming, 750);
      }

      function startHoming() {
        if (!window.confirm('确认执行回零（$H）？机器将向限位方向运动。')) return;
        if (!sendCommand('$H')) return;
        homeReady = false;
        homingRequested = true;
        homeDeadline = Date.now() + 120000;
        setHomeReady(false, '正在回零，完成前不能开始雕刻。');
        pollHoming();
      }

      function handleMachineMessage(message) {
        updateRealtimeMotionState(message);
        updateJobProgressFromStatus(message);
        if (message.includes('[POS:')) {
          if (message.includes('state=known')) {
            homingRequested = false;
            window.clearTimeout(homePollTimer);
            setHomeReady(true, '位置已确认，可以开始雕刻。');
          } else if (!homingRequested) {
            setHomeReady(false, '位置未知或已失效；请先执行回零（$H）。');
          }
        }
        if (message.includes('[HOMING:state=faulted')) {
          failHoming('回零失败；请检查限位开关与机器状态后重试。');
        }
        if (message.includes('[HOMING:state=completed')) requestPositionStatus();
      }

      function setFileNotice(text, error = false) {
        fileNotice.textContent = text;
        fileNotice.className = `notice${error ? ' error' : ''}`;
      }

      function formatSize(bytes) {
        if (!Number.isFinite(bytes) || bytes < 0) return '目录';
        const units = ['B', 'KB', 'MB', 'GB'];
        let value = bytes;
        let unit = 0;
        while (value >= 1024 && unit + 1 < units.length) { value /= 1024; unit += 1; }
        return `${value.toFixed(unit ? 2 : 0)} ${units[unit]}`;
      }

      async function responseError(response) {
        const text = await response.text();
        throw new Error(text || `${response.status} ${response.statusText}`);
      }

      function renderFiles(entries) {
        fileList.replaceChildren();
        for (const entry of entries) {
          const item = document.createElement('li');
          item.className = 'file-row';
          const name = document.createElement('span');
          name.className = 'file-name';
          name.textContent = entry.path;
          name.title = entry.path;
          const size = document.createElement('span');
          size.className = 'file-size';
          size.textContent = formatSize(entry.size);
          const actions = document.createElement('span');
          actions.className = 'file-actions';
          if (!entry.directory) {
            const download = document.createElement('a');
            download.href = `/api/v1/storage/file?path=${encodeURIComponent(entry.path)}`;
            download.textContent = '下载';
            download.className = 'subtle';
            actions.append(download);
            const engrave = document.createElement('button');
            engrave.type = 'button';
            engrave.className = 'engrave';
            engrave.dataset.engrave = '1';
            engrave.textContent = '雕刻';
            engrave.disabled = !homeReady || !socket || socket.readyState !== WebSocket.OPEN;
            engrave.addEventListener('click', () => startEngraving(entry.path));
            actions.append(engrave);
            const remove = document.createElement('button');
            remove.type = 'button';
            remove.className = 'danger';
            remove.textContent = '删除';
            remove.addEventListener('click', () => deleteFile(entry.path));
            actions.append(remove);
          }
          item.append(name, size, actions);
          fileList.append(item);
        }
      }

      async function refreshFiles() {
        refreshFilesButton.disabled = true;
        setFileNotice('正在读取 SD 卡…');
        try {
          const response = await fetch('/api/v1/storage/list?path=%2F', { cache: 'no-store' });
          if (!response.ok) await responseError(response);
          const entries = await response.json();
          renderFiles(entries);
          setFileNotice(`共 ${entries.length} 个项目。`);
        } catch (error) {
          fileList.replaceChildren();
          setFileNotice(`无法读取文件列表：${error.message}`, true);
        } finally {
          refreshFilesButton.disabled = false;
        }
      }

      async function deleteFile(path) {
        if (!window.confirm(`确认删除 ${path}？`)) return;
        setFileNotice(`正在删除 ${path}…`);
        try {
          const response = await fetch(`/api/v1/storage/file?path=${encodeURIComponent(path)}`, { method: 'DELETE' });
          if (!response.ok) await responseError(response);
          await refreshFiles();
        } catch (error) {
          setFileNotice(`删除失败：${error.message}`, true);
        }
      }

      function startEngraving(path) {
        if (!homeReady) {
          setFileNotice('开始雕刻前必须先完成回零（$H）。', true);
          return;
        }
        if (!window.confirm(`确认开始雕刻 ${path}？`)) return;
        if (!sendCommand(`$SD/RUN=${path}`)) {
          setFileNotice('WebSocket 未连接，无法开始雕刻。', true);
          return;
        }
        beginJobProgressMonitoring();
        setFileNotice(`已请求开始雕刻：${path}`);
      }

      function holdEngraving() {
        if (!sendCommand('!')) return;
        jobHoldButton.disabled = true;
      }

      function resumeEngraving() {
        if (!sendCommand('~')) return;
        jobResumeButton.disabled = true;
      }

      function resetEngraving() {
        if (!window.confirm('确认终止当前作业并复位？当前位置将失效，需要重新回零。')) return;
        if (!sendCommand(String.fromCharCode(0x18))) return;
        jobHoldButton.disabled = true;
        jobResumeButton.disabled = true;
        jobResetButton.disabled = true;
      }

      uploadForm.addEventListener('submit', async (event) => {
        event.preventDefault();
        const file = uploadFile.files[0];
        if (!file) return;
        const targetPath = `/${file.name}`;
        uploadButton.disabled = true;
        setFileNotice(`正在上传 ${file.name}…`);
        try {
          const response = await fetch(`/api/v1/storage/file?path=${encodeURIComponent(targetPath)}`, {
            method: 'PUT',
            headers: { 'Content-Type': file.type || 'application/octet-stream' },
            body: file,
          });
          if (!response.ok) await responseError(response);
          uploadFile.value = '';
          await refreshFiles();
        } catch (error) {
          setFileNotice(`上传失败：${error.message}`, true);
        } finally {
          uploadButton.disabled = false;
        }
      });

      function connect() {
        const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
        socket = new WebSocket(`${scheme}://${location.hostname}:8849/`);
        socket.binaryType = 'arraybuffer';
        socket.onopen = () => {
          homingRequested = false;
          setConnection(true);
          setHomeReady(false, '正在读取位置状态；未知位置时必须先回零。');
          append('[WebSocket connected]');
          requestPositionStatus();
          startProgressPolling();
        };
        socket.onmessage = (event) => {
          const message = typeof event.data === 'string' ? event.data : new TextDecoder().decode(event.data);
          handleMachineMessage(message);
          if (!message.startsWith('<')) append(message);
        };
        socket.onerror = () => append('[WebSocket error]');
        socket.onclose = () => {
          homingRequested = false;
          window.clearTimeout(homePollTimer);
          stopProgressPolling();
          sdJobActive = false;
          setHomeReady(false, '连接中断；重连后会重新读取位置状态。');
          setConnection(false);
          append('[WebSocket disconnected; retrying]');
          window.setTimeout(connect, 2000);
        };
      }

      form.addEventListener('submit', (event) => {
        event.preventDefault();
        const command = input.value.trim();
        if (!command || !sendCommand(command)) return;
        input.value = '';
      });

      homeButton.addEventListener('click', startHoming);
      jobHoldButton.addEventListener('click', holdEngraving);
      jobResumeButton.addEventListener('click', resumeEngraving);
      jobResetButton.addEventListener('click', resetEngraving);

      connect();
      refreshFiles();
    })();
  </script>
</body>
</html>
)HTML";

}  // namespace ray2::httpd
