"use strict";

// GenericViewer just displays message fields and values in a table.
// It can be used on any ROS type.

class NaviCtrlViewer extends Viewer_NaviCtrl {
  /**
    * Gets called when Viewer is first initialized.
    * @override
  **/
  onCreate() {
    
    this.wrapper = $('<div></div>')
        .css({
            "position": "relative",
            "width": "100%",
        })
        .appendTo(this.card.content);
    
    this.wrapper2 = $('<div></div>')
        .css({
            "width": "100%",
            "padding-bottom": "80%",
            "position": "relative",
            "overflow-x": "hidden",
            "overflow-y": "hidden",
        })
        .appendTo(this.wrapper);

    // actual log container, put it inside wrapper2
    this.processTable = $('<table></table>')
    .addClass('mdl-data-table')
    .addClass('mdl-data-table-compact')
    .addClass('mdl-js-data-table')
        .css({
            "table-layout": "fixed",
            "position": "absolute",
            "width": "100%",
            "height": "100%",
            "font-size": "10pt",
            "line-height": "1.4em",
            "overflow-y": "hidden",
            "overflow-x": "hidden",
        })
        .appendTo(this.wrapper2);

    super.onCreate();
  }

  onData(data) {
    this.card.title.text("Robot Navi Control");    

    // init_ros();

    this.processTable.innerHTML = "";
      
      let html = "";

      html += "<link rel=\"stylesheet\" href=\"//code.jquery.com/ui/1.13.1/themes/base/jquery-ui.css\">"
      html += "<link rel=\"stylesheet\" href=\"/resources/demos/style.css\">"
      html += "<script src=\"https://code.jquery.com/jquery-3.6.0.js\"></script>"
      html += "<script src=\"https://code.jquery.com/ui/1.13.1/jquery-ui.js\"></script>"

      html += "<tr valign=\"top\"> <td colspan=\"3\"><label id=\"cmdVelStatusMessage\"></label><br><br>"
      html += "<b><label id=\"navInstructions\"></label></b></td></tr>"


      html += "<tr><td colspan=\"3\" style=\"text-align: center;\"> <button  id =\"go_forward\" style=\"font-size: 10pt;\" onTouchEnd=\"timedStopRobot();\" onTouchStart=\"setSpeed('forward');clearTimedStop();\" onClick=\"setSpeed('forward');\" alt=\"\" align=\"middle\" class=\"ui-button ui-widget ui-corner-all\">Go Forward</button></td></tr>"

      html += "<tr>"
      html += "<td width=\"33%\" style=\"text-align: center;\"> <button  id =\"turn_left\" style=\"font-size: 10pt;\" onTouchEnd=\"timedStopRobot();\" onTouchStart=\"setSpeed('left');clearTimedStop();\" onClick=\"setSpeed('left');\" alt=\"\" align=\"middle\" class=\"ui-button ui-widget ui-corner-all\">Turn Left</button></td>"
      html += "<td width=\"33%\" style=\"text-align: center;\"> <button  id =\"stop\" style=\"font-size: 10pt;\" onClick=\"stopRobot('forward');\" alt=\"\" align=\"middle\" class=\"ui-button ui-widget ui-corner-all\">Stop</button></td>"
      html += "<td width=\"33%\" style=\"text-align: center;\"> <button  id =\"turn_right\" style=\"font-size: 10pt;\" onTouchEnd=\"timedStopRobot();\" onTouchStart=\"setSpeed('right');clearTimedStop();\" onClick=\"setSpeed('right');\" alt=\"\" align=\"middle\" class=\"ui-button ui-widget ui-corner-all\">Turn Right</button></td>"
      html += "</tr>"

      html += "<tr><td colspan=\"3\" style=\"text-align: center;\"> <button  id =\"go_backward\" style=\"font-size: 10pt;\" onTouchEnd=\"timedStopRobot();\" onTouchStart=\"setSpeed('backward');clearTimedStop();\" onClick=\"setSpeed('backward');\" alt=\"\" align=\"middle\" class=\"ui-button ui-widget ui-corner-all\">Go Backward</button></td></tr>"

      html += "<tr>"
      html += "<td align=\"right\"><span style=\"font-size: 10pt;\">Max Linear Speed:</span></td>"
      html += "<td width=\"60%\" align=\"right\"><input type=\"range\" id=\"maxLinearSpeed\" min=\"0.01\" max=\"0.5\" step=\"0.01\" value=\"0.2\" onChange=\"writeStatusMessage('maxLinearSpeedDisplay', this.value);\" "
      html += " onMouseUp=\"setMaxLinearSpeed(this.value);\" onInput=\"maxLinearSpeedDisplay.value=this.value;\"></td>"
      html += "<td align=\"right\"><span style=\"font-weight: bold;\"><output id=\"maxLinearSpeedDisplay\" size=\"4\"></output></span></td>"
      html += "</tr>"

      html += "<tr>"
      html += "<td align=\"right\"><span style=\"font-size: 10pt;\">Max Angular Speed:</span></td>"
      html += "<td width=\"60%\" align=\"right\"><input type=\"range\" id=\"maxAngularSpeed\" min=\"0.01\" max=\"2.0\" step=\"0.01\" value=\"1.0\" onChange=\"writeStatusMessage('maxAngularSpeedDisplay', this.value);\" "
      html += " onMouseUp=\"setMaxAngularSpeed(this.value);\" onInput=\"maxAngularSpeedDisplay.value=this.value;\"></td>"
      html += "<td align=\"right\"><span style=\"font-weight: bold;\"><output id=\"maxAngularSpeedDisplay\" size=\"4\"></output></span></td>"
      html += "</tr>"


        this.processTable[0].innerHTML = html;

  }
}

NaviCtrlViewer.friendlyName = "Raw data";

NaviCtrlViewer.supportedTypes = [
    "*",
];

Viewer.registerViewer(NaviCtrlViewer);