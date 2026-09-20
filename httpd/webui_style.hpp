#pragma once

namespace laserctl::httpd {

constexpr char kWebUiStyle[] = R"CSS(    :root { color-scheme: dark; font-family: system-ui, sans-serif; background: #10151d; color: #edf2f7; }
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
    .upload-progress { flex: 1 0 100%; margin: 2px 0 4px; }
    .upload-progress[hidden] { display: none; }
    .upload-progress progress { width: 100%; height: 12px; margin-top: 6px; accent-color: #5b9ee1; }
    .wifi-form { display: grid; grid-template-columns: minmax(110px, .35fr) 1fr; gap: 10px; align-items: center; }
    .wifi-form label { color: #9cacbd; font-size: .9rem; }
    .wifi-form select { padding: 10px; border: 1px solid #445468; border-radius: 6px; background: #0b1017; color: inherit; }
    .wifi-form .wifi-actions { grid-column: 1 / -1; display: flex; gap: 8px; justify-content: end; }
    .api { display: grid; grid-template-columns: max-content 1fr; gap: 8px 14px; margin: 0; }
    .api dt { color: #8ea6bd; }
    .api dd { margin: 0; })CSS";

}  // namespace laserctl::httpd
