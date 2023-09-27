"use strict";

// Get elements
const sliderP = document.getElementById("slider-p");
const sliderPValue = document.getElementById("slider-p-value");
const sliderI = document.getElementById("slider-i");
const sliderIValue = document.getElementById("slider-i-value");
const sliderD = document.getElementById("slider-d");
const sliderDValue = document.getElementById("slider-d-value");
const sliderSetpoint = document.getElementById("slider-setpoint");
const sliderSetpointValue = document.getElementById("slider-setpoint-value");
const inputsCard = document.getElementById("inputs");
const macInput = document.getElementById("mac-input");
const submitButton = document.getElementById("submit-button");
const deviceList = document.getElementById("slave-device-list");

const sliders = [sliderP, sliderI, sliderD, sliderSetpoint];

// Websocket variables
const gateway = `ws://${window.location.hostname}/ws`;
let websocket;

// Callback for slider
const updateSliders = function () {
  const xhr = new XMLHttpRequest();
  const params = new URLSearchParams();
  for (let slider of sliders) {
    params.append("slider-id", slider.id);
    params.append("slider-value", slider.value);
    document.getElementById(`${slider.id}-value`).innerText = slider.value;
  }
  xhr.open("POST", "/set-sliders", true);
  xhr.setRequestHeader("Content-type", "application/x-www-form-urlencoded");
  xhr.send(params);
};

// Callback for sending MAC address to the server
const addMac = function () {
  const xhr = new XMLHttpRequest();
  const mac = macInput.value;
  const regex = /^([0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2})$/;
  if (regex.test(mac)) {
    const params = new URLSearchParams();
    params.append("mac", mac);
    xhr.open("POST", "/add-mac", true);
    xhr.setRequestHeader("Content-type", "application/x-www-form-urlencoded");
    xhr.send(params);
    macInput.value = "";
  } else {
    macInput.classList.add("mac-nok");
  }
};

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

// Handle data that received on websocket
function onMessage(event) {
  console.log(event.data);
  const data = JSON.parse(event.data);

  const keys = Object.keys(data);

  for (let key of keys) {
    const sliders = [];
    const macAddresses = [];
    console.log(key);
    if (key.includes("slider")) {
      sliders.push(key);
    } else if (key.includes("broadcastAddress")) {
    }

    if (key.includes("slider")) {
      document.getElementById(`${key}-value`).innerText = data[key];
      document.getElementById(`${key}`).value = data[key];
    } else if (key.includes("broadcastAddress") && data[key] !== "") {
      const addressElement = document.createElement("p");
      addressElement.innerText = data[key];
      deviceList.appendChild(addressElement);
    }
  }
}

// Event listeners
// Send slider value to the server on change
inputsCard.addEventListener("input", updateSliders);

// Send MAC address to the server
submitButton.addEventListener("click", function (e) {
  e.preventDefault();
  addMac();
});

// Clear "invalid-mac" format
macInput.addEventListener("input", function () {
  macInput.classList.remove("mac-nok");
});
