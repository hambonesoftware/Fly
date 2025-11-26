/*
 * UI Helper Module
 *
 * This module provides simple helper functions for updating status
 * messages and toggling a busy state in the flythrough planner
 * frontend.  Centralising these functions allows for reuse across
 * different scripts and keeps DOM manipulation consistent.  If
 * additional UI helpers are needed in the future they should be
 * defined here.
 */

/**
 * Update the status message displayed to the user.
 *
 * The status is shown in the element with id "status".  The text
 * colour is green for normal messages and red for error messages.
 *
 * @param {string} message - The message to display
 * @param {boolean} [isError=false] - Whether to mark the message as an error
 */
export function setStatus(message, isError = false, isWarning = false) {
  const statusDiv = document.getElementById('status');
  if (statusDiv) {
    statusDiv.textContent = message;
    let color = '#a0e8af'; // default success green
    if (isError) {
      color = '#ff8080'; // red
    } else if (isWarning) {
      color = '#ffd27f'; // amber/yellow
    }
    statusDiv.style.color = color;
  }
}

/**
 * Indicate whether the application is busy.
 *
 * When busy, interactions such as buttons could be disabled or a
 * spinner could be shown.  For this simple implementation we
 * toggle a "busy" CSS class on the body element.  You can style
 * this class in your CSS to provide visual feedback (e.g. show a
 * spinner or dim the UI).
 *
 * @param {boolean} isBusy - True to mark the app as busy, false to clear
 */
export function setBusy(isBusy) {
  const body = document.body;
  if (!body) return;
  if (isBusy) {
    body.classList.add('busy');
  } else {
    body.classList.remove('busy');
  }
}
