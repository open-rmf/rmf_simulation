import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
import QtQuick.Window 2.2


Window {
    title: qsTr("Front view")
    id: testview
    x: 100
    y: 100
    readonly property int defaultWidth: 1600
    readonly property int defaultHeight: 900
    width: defaultWidth
    height: defaultHeight
    minimumWidth: 320
    minimumHeight: 180
    visible: true
    flags: Qt.Window

    Shortcut {
    sequence: "Ctrl+F"
    onActivated: showFullScreen()
    }

    Shortcut {
        sequence: "Esc"
        onActivated: show()
    }

    Connections {
      target: multiview
      function onNewFrontImage() {
        front.reload();
      }
    }

    Image {
      id: front
      fillMode: Image.PreserveAspectFit
      anchors.fill: parent
      function reload() {
        source = "image://front_view/" + Math.random().toString(36).substr(2, 5);
      }
    }
}