"use strict";

const macText = document.getElementById("mac");

// Get MAC address from server
const xhr = new XMLHttpRequest();
xhr.onreadystatechange = function () {
  if (this.readyState == 4 && this.status == 200) {
    const macAddress = this.responseText;
    macText.innerText = macAddress;
  }
};
xhr.open("GET", "/get-mac", true);
xhr.send();
