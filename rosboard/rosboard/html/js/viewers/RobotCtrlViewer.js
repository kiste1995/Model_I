"use strict";

// RobotCtrlViewer just displays message fields and values in a table.
// It can be used on any ROS type.

class RobotCtrlViewer extends Viewer_RobotMan {
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

    this.script_wrapper = $('<script></script')
        .appendTo(this.card.content);
    
    this.wrapper2 = $('<div></div>')
        .css({
            "width": "100%",
            "position": "relative",
            "overflow-x": "hidden",
            "overflow-y": "hidden",
        })
        .appendTo(this.wrapper);

    this.processTable = $('<table></table>')
    .addClass('mdl-data-table')
    .addClass('mdl-js-data-table')
        .css({
            "table-layout": "fixed",
            "width": "100%",
            "line-height": "1.4em",
            "overflow-y": "hidden",
            "overflow-x": "hidden",
        })
        .appendTo(this.wrapper2);

    super.onCreate();
  }

  onData(data) {
    this.card.title.text("Robot Control Viewer");    

    this.script_wrapper.innerHTML = "";

    let html_script="";

    html_script += "if(initOK == false)"
    html_script += "{ init(); init_ros(this.topicType);"
    html_script += "initOK = true;  }"

    html_script += " runRobotCtrl = true;  "

    html_script += "var bs1 = new ButtonStrip({ id: \"buttonStrip1\" });  "
    html_script += "bs1.addButton(\"Run CAM View\", true, \"click\", function() { console.log('Hallo Welt!'); });  "
    html_script += "bs1.addButton(\"Stop CAM View\", false, \"click\", function(){ console.log('Test'); });  "
    html_script += "bs1.append(\"#camview_button\");  "

    this.script_wrapper[0].innerHTML = html_script;

    this.processTable.innerHTML = "";
      
    let html = "";

      html += "<tr align =\"center\"><td colspan=\"3\" style=\"text-align: center; font-weight: bold; font-size: 16pt;\">Robot Control - TeleOP</td></tr>"

      html += "<tr valign=\"top\"> <td style=\"text-align: right;\">"
      html += "<label class=\"toggle-switchy\" for=\"teleop_ctrl\" data-size=\"lg\" data-style=\"rounded\" data-color=\"green\">"
      html += "						<input type=\"checkbox\" id=\"teleop_ctrl\" onclick='setSendCmd(event)' >"
      html += "						<span class=\"toggle\">"
      html += "						    <span class=\"switch\"></span>"
      html += "					</span></label>"
      html += "</td>"
      html += "<td colspan=\"2\" style=\"text-align: right;\"><label id=\"cmdVelStatusMessage\"></label></td></tr>"
      html += "<tr align =\"center\"><td colspan=\"3\" style=\"text-align: center; font-weight: bold; font-size: 16pt;\"><b><label id=\"navInstructions\"></label></td></tr>"

    //   html += "<tr valign=\"top\"> <td colspan=\"3\" style=\"text-align: right;\"><label id=\"cmdVelStatusMessage\"></label><br><br>"
    //   html += "<b><label id=\"navInstructions\"></label></b></td></tr>"

      html += "<tr><td colspan=\"3\" style=\"text-align: center;\"> <button  id =\"go_forward\" style=\"font-size: 10pt;\" onTouchEnd=\"timedStopRobot();\" onTouchStart=\"setSpeed('forward');clearTimedStop();\" onClick=\"setSpeed('forward');\" alt=\"\" align=\"middle\" class=\"forward\">Go Forward</button></td></tr>"

      html += "<tr>"
      html += "<td width=\"33%\" style=\"text-align: center;\"> <button  id =\"turn_left\" style=\"font-size: 10pt;\" onTouchEnd=\"timedStopRobot();\" onTouchStart=\"setSpeed('left');clearTimedStop();\" onClick=\"setSpeed('left');\" alt=\"\" align=\"middle\" class=\"left\">Turn Left</button></td>"
      html += "<td width=\"33%\" style=\"text-align: center;\"> <button  id =\"stop\" style=\"font-size: 10pt;\" onClick=\"stopRobot('forward');\" alt=\"\" align=\"middle\" class=\"stop\">Stop</button></td>"
      html += "<td width=\"33%\" style=\"text-align: center;\"> <button  id =\"turn_right\" style=\"font-size: 10pt;\" onTouchEnd=\"timedStopRobot();\" onTouchStart=\"setSpeed('right');clearTimedStop();\" onClick=\"setSpeed('right');\" alt=\"\" align=\"middle\" class=\"right\">Turn Right</button></td>"
      html += "</tr>"

      html += "<tr><td colspan=\"3\" style=\"text-align: center;\"> <button  id =\"go_backward\" style=\"font-size: 10pt;\" onTouchEnd=\"timedStopRobot();\" onTouchStart=\"setSpeed('backward');clearTimedStop();\" onClick=\"setSpeed('backward');\" alt=\"\" align=\"middle\" class=\"backward\">Go Backward</button></td></tr>"

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

      html += "<tr align =\"center\"><td colspan=\"3\" style=\"text-align: center; font-weight: bold; font-size: 16pt;\">Air Purifier</td></tr>"

      html += "<tr><td style=\"text-align: center;\">"
      html += "<label class=\"toggle-switchy\" for=\"air_purifier\" data-size=\"lg\" data-style=\"rounded\" data-color=\"green\">"
      html += "						<input type=\"checkbox\" id=\"air_purifier\" onclick='run_pubAP(event)' >"
      html += "						<span class=\"toggle\">"
      html += "						    <span class=\"switch\"></span>"
      html += "					</span></label>"
      html += "</td>"
      html += "<td colspan=\"2\"><input id =\"APVal\" onclick='run_knobAP()' type=\"number\" name=\"air_purifier\" turn=\"1\"  value=\"5\" min=\"0\" max=\"400\" step=\"10\" size=\"3\" disabled=\"disabled\" /></td>"
      html += "</tr>"


      html += "<tr align =\"center\"><td colspan=\"3\" style=\"text-align: center; font-weight: bold; font-size: 16pt;\">UVC ON/OFF</td></tr>"

      html += "<tr><td colspan=\"3\" style=\"text-align: center;\">"
      html += "<label class=\"toggle-switchy\" for=\"uvc\" data-size=\"lg\" data-style=\"rounded\" data-color=\"green\">"
      html += "						<input type=\"checkbox\" id=\"uvc\" onclick='run_pubUVC(event)' >"
      html += "						<span class=\"toggle\">"
      html += "						    <span class=\"switch\"></span>"
      html += "					</span></label>"
      html += "</td></tr>"

      html += "<tr align =\"center\"><td colspan=\"3\" style=\"text-align: center; font-weight: bold; font-size: 16pt;\">LED-Bar Control</td></tr>"
    
      html += "<tr>"
      html += "<td class=\"mdl-data-table__cell--non-numeric\" style=\"width: 20%; font-weight: bold;\">"
      html += "<select id=\"front_ledbar\" style=\"font-weight: bold; font-size: 14pt;\">"
      html += "    <option value>Front LED</option>"
      html += "    <option value=\"off\">Off</option>"
      html += "    <option value=\"on\">On</option>"
      html += "    <option value=\"blink\">Blink</option>"
      html += "    <option value=\"fblink\">FBlink</option>"
      html += "    <option value=\"fade\">Fade</option>"
      html += "    <option value=\"sweep\">Sweep</option>"
      html += "    <option value=\"fsweep\">FSweep</option>"
      html += "    <option value=\"stay\">Stay</option>"
      html += "  </select>&nbsp;&nbsp;&nbsp;"   
      html += "</td>"

      html += "<td class=\"mdl-data-table__cell--non-numeric monospace\" style=\"text-align: left;\">"
      html += "<label style=\" font-size: 11pt; \" for=\"fledbar\">Front Color</label></td>"
      html += "<td class=\"mdl-data-table__cell--non-numeric monospace\" style=\"text-align: left;\">"
      html += "<input type=\"color\" id=\"fledbar\" name=\"fledbar\" value=\"#e66465\"></td>"
      html += "</tr>"

      html += "<tr>"
      html += "<td class=\"mdl-data-table__cell--non-numeric\" style=\"width: 20%; font-weight: bold;\">"
      html += "<select id=\"rear_ledbar\" style=\"font-weight: bold; font-size: 14pt;\">"
      html += "    <option value>Rear LED</option>"
      html += "    <option value=\"off\">Off</option>"
      html += "    <option value=\"on\">On</option>"
      html += "    <option value=\"blink\">Blink</option>"
      html += "    <option value=\"fblink\">FBlink</option>"
      html += "    <option value=\"fade\">Fade</option>"
      html += "    <option value=\"sweep\">Sweep</option>"
      html += "    <option value=\"fsweep\">FSweep</option>"
      html += "    <option value=\"stay\">Stay</option>"
      html += "  </select>&nbsp;&nbsp;&nbsp;"   
      html += "</td>"
      
      html += "<td class=\"mdl-data-table__cell--non-numeric monospace\" style=\"text-align: left;\">"
      html += "<label style=\" font-size: 11pt; \" for=\"rledbar\">Rear Color</label></td>"
      html += "<td class=\"mdl-data-table__cell--non-numeric monospace\" style=\"text-align: left;\">"
      html += "<input type=\"color\" id=\"rledbar\" name=\"rledbar\" value=\"#e66465\">&nbsp;"
      html += "<button class=\"success\" onclick='run_LEDCtrl()'> Send</button>"
      html +="</td>"
      html += "</tr>" 

      html += "<tr align =\"center\"><td colspan=\"3\" style=\"text-align: center; font-weight: bold; font-size: 16pt;\">Launch topic of USB CAM </td></tr>"
      html += "<tr align =\"center\">"
      html += "<td colspan=\"3\" style=\"text-align: center; font-weight: bold;\">"
      html += "<label class=\"toggle-switchz\" for=\"camview_topic\" data-size=\"xl\" data-style=\"square\" data-color=\"green\">"
      html += "						<input type=\"checkbox\" id=\"camview_topic\" onclick='sendCAMViewTopic(event)' >"
      html += "						<span class=\"toggle\">"
      html += "						    <span class=\"switch\"></span>"
      html += "					</span></label>"
      // html += "<div id=\"camview_button\"></div>"
      html += "</td>"
      html += "</tr>"

      this.processTable[0].innerHTML = html;

  }
}

RobotCtrlViewer.friendlyName = "Robot Control";

RobotCtrlViewer.supportedTypes = [
    "*",
];

Viewer.registerViewer(RobotCtrlViewer);