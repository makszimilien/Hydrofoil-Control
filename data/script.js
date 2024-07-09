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
const sliderFactor = document.getElementById("slider-factor");
const sliderFactorValue = document.getElementById("slider-factor-value");
const sliderEnable = document.getElementById("slider-enable");
const sliderEableValue = document.getElementById("slider-enable-value");
const sliderServoMin = document.getElementById("slider-servo-min");
const sliderServoMinValue = document.getElementById("slider-servo-min-value");
const sliderServoMax = document.getElementById("slider-servo-max");
const sliderServoMaxValue = document.getElementById("slider-servo-max-value");
const inputsCard = document.getElementById("inputs");
const macInput = document.getElementById("mac-input");
const submitButton = document.getElementById("submit-button");
const deviceList = document.getElementById("slave-device-list");
const processValuesCard = document.getElementById("process-values");
const wifiOffButton = document.getElementById("wifi-off-button");
const boardSelector = document.getElementById("board-selector");
const removeSlavesButton = document.getElementById("remove-slaves-button");

const sliders = [
  sliderP,
  sliderI,
  sliderD,
  sliderSetpoint,
  sliderFactor,
  sliderEnable,
  sliderServoMin,
  sliderServoMax,
];

// Callback for slider
const updateSliders = function (event) {
  const xhr = new XMLHttpRequest();
  const params = new URLSearchParams();

  for (let slider of sliders) {
    params.append(slider.id, slider.value);
    document.getElementById(`${slider.id}-value`).innerText = slider.value;
  }
  params.append("servo-target", `${boardSelector.value}-${event.target.id}`);
  params.append("board-selector", boardSelector.value);
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
    setTimeout(function () {
      location.reload(true);
    }, 300);
  } else {
    macInput.classList.add("mac-nok");
  }
};

// Callback for turning the Wifi off
const turnWifiOff = function (event) {
  const xhr = new XMLHttpRequest();
  const params = new URLSearchParams();
  params.append("wifi-off", "true");
  xhr.open("POST", "/wifi-off", true);
  xhr.setRequestHeader("Content-type", "application/x-www-form-urlencoded");
  xhr.send(params);
};

// Callback for removing slave devices
const removeSlaves = function (event) {
  const xhr = new XMLHttpRequest();
  const params = new URLSearchParams();
  params.append("remove-slaves", "true");
  xhr.open("POST", "/remove-slaves", true);
  xhr.setRequestHeader("Content-type", "application/x-www-form-urlencoded");
  xhr.send(params);
  setTimeout(function () {
    location.reload(true);
  }, 300);
};

// Select board to control
const selectBoard = function (event) {
  const xhr = new XMLHttpRequest();
  const params = new URLSearchParams();
  params.append("board-selector", boardSelector.value);
  xhr.open("POST", "/select-board", true);
  xhr.setRequestHeader("Content-type", "application/x-www-form-urlencoded");
  xhr.send(params);
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

      boardSelector.value = data["board-selector"];

      console.log(data);
    })
    .catch((error) => {
      console.error("Fetch error:", error);
    });
};

// Get MAC addresses
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
        const addressElement = document.createElement("p");
        addressElement.innerText = data[address];
        deviceList.appendChild(addressElement);
      });
    })
    .catch((error) => {
      console.error("Fetch error:", error);
    });
};

// Get process values
const getValues = function () {
  fetch("/get-values")
    .then((response) => {
      if (!response.ok) {
        throw new Error("Response not OK");
      }
      return response.json();
    })
    .then((data) => {
      processValuesCard.innerHTML = "";
      Object.keys(data).forEach((valueKey) => {
        const valueElement = document.createElement("p");
        valueElement.innerText = `${valueKey}: ${Math.floor(data[valueKey])}`;
        processValuesCard.appendChild(valueElement);
      });
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

function selectBoardCb() {
  selectBoard();
  getSettings();
}
// Interval timer for getting values from the server
// setInterval(getValues, 500);

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

wifiOffButton.addEventListener("click", turnWifiOff);

removeSlavesButton.addEventListener("click", removeSlaves);

boardSelector.addEventListener("change", selectBoardCb);
