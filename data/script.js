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
const sliderEnable = document.getElementById("slider-enable");
const sliderEableValue = document.getElementById("slider-setpoint-value");
const sliderServoMin = document.getElementById("slider-servo-min");
const sliderServoMinValue = document.getElementById("slider-servo-min-value");
const sliderServoMax = document.getElementById("slider-servo-max");
const sliderServoMaxValue = document.getElementById("slider-servo-max-value");
const inputsCard = document.getElementById("inputs");
const macInput = document.getElementById("mac-input");
const submitButton = document.getElementById("submit-button");
const deviceList = document.getElementById("slave-device-list");
const processValuesCard = document.getElementById("process-values");

const sliders = [
  sliderP,
  sliderI,
  sliderD,
  sliderSetpoint,
  sliderEnable,
  sliderServoMin,
  sliderServoMax,
];

// Websocket variables
const gateway = `ws://${window.location.hostname}/ws`;
let websocket;
let responseReceived = false;

// Callback for slider
const updateSliders = function (event) {
  const xhr = new XMLHttpRequest();
  const params = new URLSearchParams();

  for (let slider of sliders) {
    params.append(slider.id, slider.value);
    document.getElementById(`${slider.id}-value`).innerText = slider.value;
  }
  params.append("target", event.target.id);
  console.log(event);
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
    // Reload page
    location.reload(true);
  } else {
    macInput.classList.add("mac-nok");
  }
};

// Get settings
const getSettings = function () {
  fetch("/get-settings")
    .then((response) => {
      if (!response.ok) {
        throw new Error("Response not OK");
      }
      return response.json();
    })
    .then((data) => {
      for (let slider of sliders) {
        slider.value = data[slider.id];
        document.getElementById(`${slider.id}-value`).innerText = slider.value;
      }

      console.log(data);
    })
    .catch((error) => {
      console.error("Fetch error:", error);
    });
};

// Get settings
const getAddresses = function () {
  fetch("/get-addresses")
    .then((response) => {
      if (!response.ok) {
        throw new Error("Response not OK");
      }
      return response.json();
    })
    .then((data) => {
      deviceList.innerHTML = "";
      Object.keys(data).forEach((address) => {
        console.log(address);
        const addressElement = document.createElement("p");
        addressElement.innerText = data[address];
        deviceList.appendChild(addressElement);
      });
      // console.log(data);
    })
    .catch((error) => {
      console.error("Fetch error:", error);
    });
};

// Get setting and MAC addresses on page load event
window.addEventListener("load", onload);

function onload() {
  getSettings();
  getAddresses();
}

// if (processValues.length > 0) {
//   processValuesCard.innerHTML = "";
//   processValues.forEach(function (valueKey) {
//     // console.log(valueKey);
//     // console.log(data[valueKey]);
//     const valueElement = document.createElement("p");
//     valueElement.innerText = `${valueKey.replace(
//       "process-value-",
//       ""
//     )}: ${Math.floor(data[valueKey])}`;
//     processValuesCard.appendChild(valueElement);
//   });
// }

// Event listeners

// Send slider value to the server on change

let debounceTimer;
const debouncedUpdateSliders = function (event) {
  clearTimeout(debounceTimer);
  debounceTimer = setTimeout(function () {
    updateSliders(event);
  }, 300); // Adjust the delay as needed
};

inputsCard.addEventListener("input", function (event) {
  debouncedUpdateSliders(event);
});

// Send MAC address to the server
submitButton.addEventListener("click", function (e) {
  e.preventDefault();
  addMac();
});

// Clear "invalid-mac" format
macInput.addEventListener("input", function () {
  macInput.classList.remove("mac-nok");
});
