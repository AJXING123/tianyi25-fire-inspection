/* 登录页脚本：根据查询参数显示错误提示，并保留登录后的回跳地址。 */
(function () {
  "use strict";

  var params = new URLSearchParams(window.location.search);

  if (params.get("error") === "1") {
    var box = document.getElementById("login-error");
    if (box) box.classList.add("show");
  }
  if (params.get("error") === "setup") {
    var setup = document.getElementById("login-setup");
    if (setup) setup.hidden = false;
  }

  var next = params.get("next") || "/";
  // 只允许站内相对路径，防开放重定向
  if (!next.startsWith("/") || next.startsWith("//")) next = "/";
  var field = document.getElementById("next-field");
  if (field) field.value = next;

  var input = document.getElementById("password");
  if (input) input.focus();
})();
