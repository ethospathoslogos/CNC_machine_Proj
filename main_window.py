import main
import json
from main import ProcessorWorker
from main import PreviewCanvas
from pathlib import Path
from typing import Any, Optional, List


from PySide6.QtCore import  QThread

from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QLabel,
    QFileDialog,
    QMessageBox,
    QTextEdit,
    QGroupBox,
    QSpacerItem,
    QSizePolicy,
    
)
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Signature Engravers Program v1")
        self.resize(1100, 700)

        # Top-level layout
        central = QWidget()
        main_layout = QHBoxLayout()
        central.setLayout(main_layout)
        self.setCentralWidget(central)

        # Left: preview canvas
        
        self.preview = PreviewCanvas()
        main_layout.addWidget(self.preview, stretch=3)

        # Right: controls & console
        right_col = QVBoxLayout()
        main_layout.addLayout(right_col, stretch=2)

        # File / selection group
        file_group = QGroupBox("File")
        fg_layout = QVBoxLayout()
        file_group.setLayout(fg_layout)
        self.lbl_file = QLabel("No file selected.")
        self.btn_open = QPushButton("Open SVG...")
        self.btn_open.clicked.connect(self.open_svg_dialog)
        # Save G-code button (disabled until processing produces G-code)
        self.btn_save = QPushButton("Save G-code...")
        self.btn_save.setEnabled(False)
        self.btn_save.clicked.connect(self.save_gcode_dialog)
        fg_layout.addWidget(self.lbl_file)
        fg_layout.addWidget(self.btn_open)
        fg_layout.addWidget(self.btn_save)
        right_col.addWidget(file_group)

        # Console / log
        console_group = QGroupBox("Console")
        c_layout = QVBoxLayout()
        console_group.setLayout(c_layout)
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        c_layout.addWidget(self.console)
        right_col.addWidget(console_group, stretch=1)

        # Status bar (simple QLabel)
        self.status_label = QLabel("Idle")
        self.status_label.setMinimumHeight(24)
        self.statusBar().addPermanentWidget(self.status_label)

        # Spacer to push controls up
        right_col.addItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))

        # State
        self.current_svg: Path = None
        self._worker: ProcessorWorker = None
        self._thread: QThread = None
        self._last_polylines: Optional[List[List[tuple]]] = None
        self._last_gcode: Optional[List[str]] = None

        # Default UI state
        self.btn_save.setEnabled(False)

    def console_append(self, text: str) -> None:
        self.console.append(text)
        # Keep last lines visible
        self.console.verticalScrollBar().setValue(self.console.verticalScrollBar().maximum())

    def open_svg_dialog(self) -> None:
        fname, _ = QFileDialog.getOpenFileName(
            self, "Open SVG file", "", "SVG Files (*.svg);;All Files (*)"
        )
        if not fname:
            return

        path = Path(fname)

        # Defensive validation
        if path.suffix.lower() != ".svg":
            QMessageBox.warning(self, "Invalid file", "Please select an .svg file.")
            return
        if not path.exists():
            QMessageBox.critical(self, "File missing", f"File not found:\n{fname}")
            return

        # Optional file size guard
        max_mb = 100
        try:
            if path.stat().st_size > max_mb * 1024 * 1024:
                ok = QMessageBox.question(
                    self,
                    "Large file",
                    f"The selected file is > {max_mb} MB. Continue?",
                    QMessageBox.Yes | QMessageBox.No,
                )
                if ok != QMessageBox.Yes:
                    return
        except Exception:
            pass

        # Update UI, preview the SVG and start processing in worker thread
        self.current_svg = path
        self.lbl_file.setText(str(path))
        # Load SVG into preview canvas (will fall back to text if QtSvg not available)
        try:
            self.preview.load_svg(str(path))
        except Exception:
            # ensure at least filename is shown
            self.preview.set_filename(path.name)
        self.console_append(f"Selected SVG: {path}")

        self.run_processor_worker(str(path))

    def run_processor_worker(self, svg_path: str, resolution: float = 0.5) -> None:
        # Disable open while processing
        self.btn_open.setEnabled(False)
        self.status_label.setText("Processing SVG...")
        self.console_append("Starting SVG -> G-code processing...")

        self._worker = ProcessorWorker(svg_path, resolution=resolution)
        self._thread = QThread()
        self._worker.moveToThread(self._thread)

        self._thread.started.connect(self._worker.run)
        self._worker.started.connect(lambda: self.console_append("Processor started"))
        self._worker.finished.connect(self.on_processing_finished)
        self._worker.error.connect(self.on_processing_error)

        # Clean up thread/worker when done
        self._worker.finished.connect(self._thread.quit)
        self._worker.error.connect(self._thread.quit)
        self._thread.finished.connect(self._thread.deleteLater)

        self._thread.start()

    def on_processing_finished(self, result: Any) -> None:
        self.btn_open.setEnabled(True)
        self.status_label.setText("SVG processed")
        # Expecting a dict with 'polylines' and 'gcode' keys (polylines may be None)
        try:
            polylines = result.get("polylines", None) if isinstance(result, dict) else None
            gcode = result.get("gcode", None) if isinstance(result, dict) else None
        except Exception:
            polylines = None
            gcode = result

        # Store for later saving
        self._last_polylines = polylines
        self._last_gcode = gcode if isinstance(gcode, list) else None

        # Display summary and contents
        if polylines is not None:
            self.console_append(f"Parsed {len(polylines)} polylines.")
            try:
                self.console_append(json.dumps(polylines[:5], indent=2))  # print first few for brevity
            except Exception:
                self.console_append(str(polylines))
        else:
            self.console_append("Polylines not available from svg_parser.")

        if self._last_gcode:
            self.console_append("Generated G-code:")
            for ln in self._last_gcode:
                self.console_append(ln)
            # Enable save button
            self.btn_save.setEnabled(True)
            QMessageBox.information(self, "Processing finished", "SVG parsed and G-code generated. See console.")
        else:
            self.console_append("No G-code generated.")
            QMessageBox.information(self, "Processing finished", "SVG parsed but no G-code produced.")

    def on_processing_error(self, message: str) -> None:
        self.btn_open.setEnabled(True)
        self.status_label.setText("Processing failed")
        self.console_append(f"Processor error: {message}")
        QMessageBox.critical(self, "Processing error", message)
        self._last_gcode = None
        self.btn_save.setEnabled(False)

    def save_gcode_dialog(self) -> None:
        if not self._last_gcode:
            QMessageBox.information(self, "No G-code", "No G-code available to save.")
            return
        suggested = "output.gcode"
        fname, _ = QFileDialog.getSaveFileName(self, "Save G-code", suggested, "G-code Files (*.gcode);;All Files (*)")
        if not fname:
            return
        try:
            with open(fname, "w", encoding="utf-8") as fh:
                for ln in self._last_gcode:
                    fh.write(ln + "\n")
            QMessageBox.information(self, "Saved", f"G-code written to: {fname}")
            self.console_append(f"G-code saved to {fname}")
        except Exception as e:
            QMessageBox.critical(self, "Save error", f"Could not save file: {e}")
            self.console_append(f"Failed to save G-code: {e}")

if __name__ == "__main__":
    import sys

    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())
