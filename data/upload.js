"use strict";

let attemptCount = 0;
const maxAttempts = 10;

function checkServerAvailability() {
  fetch("/")
    .then((response) => {
      if (!response.ok) {
        throw new Error("Server not available");
      }
      return response.text();
    })
    .then((data) => {
      document.getElementById("modalOverlay").style.display = "none";
      attemptCount = 0; // Reset counter if the server is available
    })
    .catch((error) => {
      document.getElementById("modalOverlay").style.display = "flex";
      attemptCount++;
      if (attemptCount >= maxAttempts) {
        location.reload(); // Reload the page after 10 failed attempts
      }
    });
}

// Check server availability every 3 seconds
setInterval(checkServerAvailability, 3000);

// Initial check
checkServerAvailability();
