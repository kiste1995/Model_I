"use strict";

// RobotStatusViewer just displays message fields and values in a table.
// It can be used on any ROS type.

// importJsOnce("js/jqwidgets/jqxgauge.js");

class RobotStatusViewer extends Viewer_RobotMan {
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

    // actual log container, put it inside wrapper2
    this.processTable = $('<table></table>')
    .addClass('mdl-data-table')
    .addClass('mdl-js-data-table')
    .css({'width': '100%', 'min-height': '10pt', 'table-layout': 'fixed' })

        .appendTo(this.wrapper2);

    super.onCreate();
  }

  onData(data) {
    this.card.title.text("Robot Status Viewer");    

    this.script_wrapper.innerHTML = "";

    let html_script="";

    html_script += "if(initOK == false)"
    html_script += "{ init(); init_ros(this.topicType);"
    html_script += "initOK = true;  }"

    html_script += " "

    html_script += "console.log(\"Inside RobotStatusViewer...\");"
    
    this.script_wrapper[0].innerHTML = html_script;


    this.processTable.innerHTML = "";
      
    let html = "";


    html += "<tr align =\"center\"><td colspan=\"2\" style=\"text-align: center; font-weight: bold; font-size: 16pt;\">Battery Status</td></tr>"

    html += "<tr>"
    html += "<td class=\"mdl-data-table__cell--non-numeric\" style=\"width: 10%; font-weight: bold; \">Voltage</td>"
    
    html += "<td class=\"mdl-data-table__cell--non-numeric monospace\" style=\"width: 90%; text-align: left; \"><label id =\"bat_vol\"></label></td>"
    html += "</tr>"

    html += "<tr>"
    html += "<td class=\"mdl-data-table__cell--non-numeric\" style=\"width: 20%; font-weight: bold; \">Current</td>"
    html += "<td class=\"mdl-data-table__cell--non-numeric monospace\" style=\"width: 80%; text-align: left; \"><label id =\"bat_cur\"></label></td>"
    html += "</tr>"


    html += "<tr>"
    html += "<td class=\"mdl-data-table__cell--non-numeric\" style=\"width: 20%; font-weight: bold; \">SOC</td>"
    html += "<td class=\"mdl-data-table__cell--non-numeric monospace\" style=\"width: 80%; text-align: left; \"><div id=\"bat_soc\"></div></td>"
    html += "</tr>"

    html += "<tr align =\"center\"><td colspan=\"2\" style=\"text-align: center; font-weight: bold; font-size: 16pt;\">Sonar Sensors</td></tr>"

    html += "<tr>"
    html += "<td class=\"mdl-data-table__cell--non-numeric\" style=\"width: 20%; font-weight: bold;\">Sonar #1</td>"
    html += "<td class=\"mdl-data-table__cell--non-numeric monospace\" style=\"width: 80%; text-align: left; \"><div id=\"sonar1\"></div></td>"
    html += "</tr>"

    html += "<tr>"
    html += "<td class=\"mdl-data-table__cell--non-numeric\" style=\"width: 20%; font-weight: bold;\">Sonar #2</td>"
    html += "<td class=\"mdl-data-table__cell--non-numeric monospace\" style=\"width: 80%; text-align: left; \"><div id=\"sonar2\"></div></td>"
    html += "</tr>"

    html += "<tr>"
    html += "<td class=\"mdl-data-table__cell--non-numeric\" style=\"width: 20%; font-weight: bold;\">Sonar #3</td>"
    html += "<td class=\"mdl-data-table__cell--non-numeric monospace\" style=\"width: 80%; text-align: left; \"><div id=\"sonar3\"></div></td>"
    html += "</tr>"

    html += "<tr>"
    html += "<td class=\"mdl-data-table__cell--non-numeric\" style=\"width: 20%; font-weight: bold;\">Sonar #4</td>"
    html += "<td class=\"mdl-data-table__cell--non-numeric monospace\" style=\"width: 80%; text-align: left; \"><div id=\"sonar4\"></div></td>"
    html += "</tr>"

    html += "<tr>"
    html += "<td class=\"mdl-data-table__cell--non-numeric\" style=\"width: 20%; font-weight: bold;\">Sonar #5</td>"
    html += "<td class=\"mdl-data-table__cell--non-numeric monospace\" style=\"width: 80%; text-align: left; \"><div id=\"sonar5\"></div></td>"
    html += "</tr>"

    html += "<tr>"
    html += "<td class=\"mdl-data-table__cell--non-numeric\" style=\"width: 20%; font-weight: bold;\">Sonar #6</td>"
    html += "<td class=\"mdl-data-table__cell--non-numeric monospace\" style=\"width: 80%; text-align: left; \"><div id=\"sonar6\"></div></td>"
    html += "</tr>"

    html += "<tr>"
    html += "<td class=\"mdl-data-table__cell--non-numeric\" style=\"width: 20%; font-weight: bold;\">Sonar #7</td>"
    html += "<td class=\"mdl-data-table__cell--non-numeric monospace\" style=\"width: 80%; text-align: left; \"><div id=\"sonar7\"></div></td>"
    html += "</tr>"

    html += "<tr>"
    html += "<td class=\"mdl-data-table__cell--non-numeric\" style=\"width: 20%; font-weight: bold;\">Sonar #8</td>"
    html += "<td class=\"mdl-data-table__cell--non-numeric monospace\" style=\"width: 80%; text-align: left; \"><div id=\"sonar8\"></div></td>"
    html += "</tr>"

    html += "<tr>"
    html += "<td class=\"mdl-data-table__cell--non-numeric\" style=\"width: 20%; font-weight: bold;\">Sonar #9</td>"
    html += "<td class=\"mdl-data-table__cell--non-numeric monospace\" style=\"width: 80%; text-align: left; \"><div id=\"sonar9\"></div></td>"
    html += "</tr>"

    html += "<tr>"
    html += "<td class=\"mdl-data-table__cell--non-numeric\" style=\"width: 20%; font-weight: bold;\">Sonar #10</td>"
    html += "<td class=\"mdl-data-table__cell--non-numeric monospace\" style=\"width: 80%; text-align: left; \"><div id=\"sonar10\"></div></td>"
    html += "</tr>"

    html += "<tr align =\"center\"><td colspan=\"2\" style=\"text-align: center; font-weight: bold; font-size: 16pt;\">Robot Heading</td></tr>"
    html += "<tr>"
    html += "<td class=\"mdl-data-table__cell--non-numeric\" >IMU&nbsp;&nbsp;&nbsp;&nbsp;<input id=\"imu\" style=\"width: 100px;\"></td>"
    html += "<td class=\"mdl-data-table__cell--non-numeric\" style=\"width: 50%;\">Angle&nbsp;&nbsp;&nbsp;<input id=\"angle\" style=\"width: 100px;\"></td>"
    html += "</tr>"
 
    this.processTable[0].innerHTML = html;
   
  }
}

RobotStatusViewer.friendlyName = "Robot Stauts";

RobotStatusViewer.supportedTypes = [
    "*",
];

Viewer.registerViewer(RobotStatusViewer);