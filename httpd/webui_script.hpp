#pragma once

namespace laserctl::httpd {

constexpr char kWebUiScript[] = R"JS(    (() => {
      const status = document.getElementById('connection');
      const position = document.getElementById('position');
      const output = document.getElementById('messages');
      const form = document.getElementById('command-form');
      const input = document.getElementById('command');
      const submit = document.getElementById('send');
      const wifiForm = document.getElementById('wifi-form');
      const wifiReadButton = document.getElementById('wifi-read');
      const wifiApplyButton = document.getElementById('wifi-apply');
      const wifiSsid = document.getElementById('wifi-ssid');
      const wifiPassword = document.getElementById('wifi-password');
      const wifiMode = document.getElementById('wifi-mode');
      const wifiNotice = document.getElementById('wifi-notice');
      const refreshFilesButton = document.getElementById('refresh-files');
      const uploadForm = document.getElementById('upload-form');
      const uploadFile = document.getElementById('upload-file');
      const uploadButton = document.getElementById('upload');
      const uploadProgressPanel = document.getElementById('upload-progress-panel');
      const uploadProgress = document.getElementById('upload-progress');
      const uploadProgressText = document.getElementById('upload-progress-text');
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
      let socketGeneration = 0;
      let socketReconnectTimer = 0;
      let pageUnloading = false;
      let homeReady = false;
      let homingRequested = false;
      let homeDeadline = 0;
      let homePollTimer = 0;
      let progressPollTimer = 0;
      let sdJobActive = false;
      let realtimeMotionState = 'Idle';
      let wifiRestartExpected = false;
      let fileTransferActive = false;
      const httpRequestTimeoutMs = 60000;
      const httpRequestQueue = [];
      let httpRequestActive = false;

      function enqueueHttpRequest(label, operation) {
        return new Promise((resolve, reject) => {
          httpRequestQueue.push({ label, operation, resolve, reject });
          serviceHttpRequestQueue();
        });
      }

      async function serviceHttpRequestQueue() {
        if (httpRequestActive || httpRequestQueue.length === 0) return;
        httpRequestActive = true;
        const request = httpRequestQueue.shift();
        try {
          request.resolve(await request.operation());
        } catch (error) {
          request.reject(error);
        } finally {
          httpRequestActive = false;
          window.setTimeout(serviceHttpRequestQueue, 0);
        }
      }

      async function fetchHttp(url, options = {}) {
        const controller = new AbortController();
        const timeout = window.setTimeout(() => controller.abort(), httpRequestTimeoutMs);
        try {
          return await fetch(url, { ...options, signal: controller.signal });
        } finally {
          window.clearTimeout(timeout);
        }
      }

      function append(message) {
        output.textContent += `${output.textContent ? '\n' : ''}${message}`;
        output.scrollTop = output.scrollHeight;
      }

      function setConnection(online) {
        status.className = `badge ${online ? 'online' : 'offline'}`;
        status.textContent = online ? 'WebSocket 已连接' : 'WebSocket 未连接';
        submit.disabled = !online;
        wifiReadButton.disabled = !online;
        wifiApplyButton.disabled = !online;
        updateHomeButton();
        updateEngraveButtons();
      }

      function updateHomeButton() {
        homeButton.disabled = !socket || socket.readyState !== WebSocket.OPEN || homingRequested;
      }

      function updateEngraveButtons() {
        const available = homeReady && !sdJobActive && !fileTransferActive && socket && socket.readyState === WebSocket.OPEN;
        document.querySelectorAll('[data-engrave]').forEach((button) => { button.disabled = !available; });
        updateJobControls();
      }

      function setFileTransferActive(active) {
        fileTransferActive = active;
        refreshFilesButton.disabled = active;
        uploadButton.disabled = active;
        uploadFile.disabled = active;
        fileList.querySelectorAll('[data-file-operation]').forEach((button) => { button.disabled = active; });
        fileList.querySelectorAll('[data-file-download]').forEach((link) => {
          if (active) {
            link.dataset.downloadUrl = link.href;
            link.removeAttribute('href');
            link.setAttribute('aria-disabled', 'true');
          } else if (link.dataset.downloadUrl) {
            link.href = link.dataset.downloadUrl;
            link.removeAttribute('aria-disabled');
          }
        });
        updateEngraveButtons();
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

      function setWifiNotice(message, error = false) {
        wifiNotice.textContent = message;
        wifiNotice.className = `notice${error ? ' error' : ''}`;
      }

      function requestWifiSettings() {
        if (!sendCommand('$STA/SSID')) return;
        sendCommand('$STA/PASSWORD');
        sendCommand('$RADIO/MODE');
        setWifiNotice('正在读取保存的无线配置。');
      }

      function wifiFieldIsValid(value, allowEmpty = false) {
        return (allowEmpty || value.length !== 0) && /^[\x20-\x7e]*$/.test(value);
      }

      function applyWifiSettings() {
        const ssid = wifiSsid.value;
        const password = wifiPassword.value;
        const mode = wifiMode.value;
        if (!wifiFieldIsValid(ssid) || ssid.length > 32 || !wifiFieldIsValid(password, true) ||
            password.length > 64) {
          setWifiNotice('SSID 需为 1-32 字符，密码需为 0-64 字符，且只能使用可打印 ASCII 字符。', true);
          return;
        }
        if (!window.confirm(`确认保存 ${mode} 配置并重启无线网络？\n\n当前页面和 WebSocket 会立即断开。\n请切换到设备重启后的网络，再用新的设备 IP 或设备 AP 地址重新打开本页。`)) {
          return;
        }
        if (!sendCommand(`$STA/SSID=${ssid}`) || !sendCommand(`$STA/PASSWORD=${password}`) ||
            !sendCommand(`$RADIO/MODE=${mode}`)) {
          setWifiNotice('WebSocket 已断开，未能保存无线配置。', true);
          return;
        }
        wifiRestartExpected = true;
        wifiApplyButton.disabled = true;
        wifiReadButton.disabled = true;
        setWifiNotice('配置已保存，正在重启无线网络。页面即将断开；请按提示重新连接。');
        sendCommand('$WIFI/BEGIN');
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
        const ssid = message.match(/\[STA:SSID=([^\]]*)\]/);
        if (ssid) wifiSsid.value = ssid[1];
        const password = message.match(/\[STA:PASSWORD=([^\]]*)\]/);
        if (password) wifiPassword.value = password[1];
        const mode = message.match(/\[RADIO:MODE=(AP_STA|STA|AP)\]/);
        if (mode) wifiMode.value = mode[1];
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

      function setUploadProgress(loaded, total, text, error = false) {
        const knownTotal = Number.isFinite(total) && total > 0;
        const safeLoaded = Math.max(0, Number.isFinite(loaded) ? loaded : 0);
        uploadProgressPanel.hidden = false;
        uploadProgress.value = knownTotal ? Math.min(1000, Math.round(safeLoaded * 1000 / total)) : 0;
        uploadProgressText.textContent = text || (knownTotal
          ? `正在发送 ${formatSize(safeLoaded)} / ${formatSize(total)}（${(safeLoaded * 100 / total).toFixed(1)}%）。`
          : `正在发送 ${formatSize(safeLoaded)}。`);
        uploadProgressText.className = `notice${error ? ' error' : ''}`;
      }

      function uploadFileToDevice(file, targetPath) {
        return enqueueHttpRequest('upload', () => new Promise((resolve, reject) => {
          const request = new XMLHttpRequest();
          request.open('PUT', `/api/v1/storage/file?path=${encodeURIComponent(targetPath)}`);
          request.timeout = httpRequestTimeoutMs;
          request.setRequestHeader('Content-Type', file.type || 'application/octet-stream');
          request.upload.addEventListener('progress', (event) => {
            setUploadProgress(event.loaded, event.lengthComputable ? event.total : file.size);
          });
          request.addEventListener('load', () => {
            if (request.status >= 200 && request.status < 300) {
              setUploadProgress(file.size, file.size, `设备已确认写入 ${formatSize(file.size)}。`);
              resolve();
              return;
            }
            reject(new Error(request.responseText || `${request.status} ${request.statusText}`));
          });
          request.addEventListener('error', () => reject(new Error('网络连接中断。')));
          request.addEventListener('abort', () => reject(new Error('上传已取消。')));
          request.addEventListener('timeout', () => reject(new Error('上传请求超时。')));
          request.send(file);
        }));
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
            download.dataset.fileDownload = '1';
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
            remove.dataset.fileOperation = '1';
            remove.addEventListener('click', () => deleteFile(entry.path));
            actions.append(remove);
          }
          item.append(name, size, actions);
          fileList.append(item);
        }
        setFileTransferActive(fileTransferActive);
      }

      async function refreshFiles() {
        refreshFilesButton.disabled = true;
        setFileNotice('正在读取 SD 卡…');
        try {
          const entries = await enqueueHttpRequest('storage-list', async () => {
            const response = await fetchHttp('/api/v1/storage/list?path=%2F', { cache: 'no-store' });
            if (!response.ok) await responseError(response);
            return response.json();
          });
          renderFiles(entries);
          setFileNotice(`共 ${entries.length} 个项目。`);
        } catch (error) {
          fileList.replaceChildren();
          setFileNotice(`无法读取文件列表：${error.message}`, true);
        } finally {
          refreshFilesButton.disabled = fileTransferActive;
        }
      }

      async function deleteFile(path) {
        if (fileTransferActive) return;
        if (!window.confirm(`确认删除 ${path}？`)) return;
        setFileNotice(`正在删除 ${path}…`);
        try {
          await enqueueHttpRequest('delete-file', async () => {
            const response = await fetchHttp(`/api/v1/storage/file?path=${encodeURIComponent(path)}`, { method: 'DELETE' });
            if (!response.ok) await responseError(response);
          });
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
        if (file.size === 0) {
          setFileNotice('不能上传空文件。', true);
          return;
        }
        const targetPath = `/${file.name}`;
        setFileTransferActive(true);
        setUploadProgress(0, file.size);
        setFileNotice(`正在上传 ${file.name}…`);
        try {
          await uploadFileToDevice(file, targetPath);
          uploadFile.value = '';
          await refreshFiles();
          setFileNotice(`上传完成：${file.name}（${formatSize(file.size)}）。`);
        } catch (error) {
          setUploadProgress(0, file.size, `上传失败：${error.message}`, true);
          setFileNotice(`上传失败：${error.message}`, true);
        } finally {
          setFileTransferActive(false);
        }
      });

      wifiReadButton.addEventListener('click', requestWifiSettings);
      wifiForm.addEventListener('submit', (event) => {
        event.preventDefault();
        applyWifiSettings();
      });

      function connect() {
        if (pageUnloading) return;
        window.clearTimeout(socketReconnectTimer);
        socketReconnectTimer = 0;
        const generation = ++socketGeneration;
        const previousSocket = socket;
        if (previousSocket && previousSocket.readyState < WebSocket.CLOSING) previousSocket.close();
        const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
        const nextSocket = new WebSocket(`${scheme}://${location.hostname}:8849/`);
        socket = nextSocket;
        nextSocket.binaryType = 'arraybuffer';
        nextSocket.onopen = () => {
          if (pageUnloading || generation !== socketGeneration || socket !== nextSocket) {
            nextSocket.close();
            return;
          }
          homingRequested = false;
          setConnection(true);
          setHomeReady(false, '正在读取位置状态；未知位置时必须先回零。');
          append('[WebSocket connected]');
          requestPositionStatus();
          startProgressPolling();
          requestWifiSettings();
        };
        nextSocket.onmessage = (event) => {
          if (pageUnloading || generation !== socketGeneration || socket !== nextSocket) return;
          const message = typeof event.data === 'string' ? event.data : new TextDecoder().decode(event.data);
          handleMachineMessage(message);
          if (!message.startsWith('<')) append(message);
        };
        nextSocket.onerror = () => {
          if (!pageUnloading && generation === socketGeneration && socket === nextSocket) {
            append('[WebSocket error]');
          }
        };
        nextSocket.onclose = () => {
          if (generation !== socketGeneration || socket !== nextSocket) return;
          socket = undefined;
          homingRequested = false;
          window.clearTimeout(homePollTimer);
          stopProgressPolling();
          sdJobActive = false;
          setHomeReady(false, '连接中断；重连后会重新读取位置状态。');
          setConnection(false);
          if (pageUnloading) return;
          if (wifiRestartExpected) {
            setWifiNotice('无线网络正在切换。请连接目标网络后，以设备新的 IP 或 AP 地址重新打开本页。');
            append('[Wi-Fi restarting; reconnect manually]');
            return;
          }
          append('[WebSocket disconnected; retrying]');
          socketReconnectTimer = window.setTimeout(() => {
            if (!pageUnloading && generation === socketGeneration) connect();
          }, 2000);
        };
      }

      window.addEventListener('pagehide', () => {
        pageUnloading = true;
        window.clearTimeout(socketReconnectTimer);
        socketReconnectTimer = 0;
        if (socket && socket.readyState === WebSocket.OPEN) socket.close();
      });

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
    })();)JS";

}  // namespace laserctl::httpd
