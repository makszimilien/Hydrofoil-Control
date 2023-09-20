"use strict";

// Get elements
const slider = document.getElementById("slider");
const sliderValue = document.getElementById("sliderValue");

// Callback for slider
const updateSlider = function (slider) {
  const xhr = new XMLHttpRequest();
  const value = slider.value;
  const output = slider.id;
  const params = new URLSearchParams();
  params.append("value", value);
  params.append("output", output);
  xhr.open("POST", "/set-output", true);
  xhr.setRequestHeader("Content-type", "application/x-www-form-urlencoded");
  xhr.send(params);
};

const gateway = `ws://${window.location.hostname}/ws`;
let websocket;
// Init web socket when the page loads
window.addEventListener("load", onload);

function onload(event) {
  initWebSocket();
}

function getReadings() {
  websocket.send("getReadings");
}

function initWebSocket() {
  console.log("Trying to open a WebSocket connection…");
  websocket = new WebSocket(gateway);
  websocket.onopen = onOpen;
  websocket.onclose = onClose;
  websocket.onmessage = onMessage;
}

// When websocket is established, call the getReadings() function
function onOpen(event) {
  console.log("Connection opened");
  getReadings();
}

function onClose(event) {
  console.log("Connection closed");
  setTimeout(initWebSocket, 2000);
}

// Function that receives the message from the ESP32 with the readings
function onMessage(event) {
  console.log(event.data);
  const data = JSON.parse(event.data);

  const keys = Object.keys(data);

  for (let i = 0; i < keys.length; i++) {
    const key = keys[i];
    document.getElementById(key).innerHTML = data[key];
    document.getElementById(`${key}Value`).innerHTML = data[key];
    console.log();
  }
}

// Update the value display when the slider value changes
slider.addEventListener("change", function () {
  updateSlider(this);
});
