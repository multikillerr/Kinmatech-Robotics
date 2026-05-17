#!/usr/bin/env python3
"""
Table model for robot program display and editing
"""

from typing import List, Any, Optional
from PyQt6.QtCore import QAbstractTableModel, Qt, QModelIndex, QVariant
from PyQt6.QtWidgets import QApplication
from PyQt6.QtGui import QColor, QBrush, QFont
from planning.data_models import ProgramRow, Pose, WeldParams


class ProgramTableModel(QAbstractTableModel):
    """Table model for displaying and editing robot program rows"""

    MOTION_TYPE_COLORS = {
        "LINEAR": {
            "row_bg": QColor(70, 120, 220, 40),
            "type_bg": QColor(70, 120, 220, 110),
            "type_fg": QColor(235, 242, 255),
        },
        "CURVE": {
            "row_bg": QColor(255, 170, 64, 45),
            "type_bg": QColor(255, 170, 64, 120),
            "type_fg": QColor(30, 18, 0),
        },
        "P2P": {
            "row_bg": QColor(46, 204, 113, 45),
            "type_bg": QColor(46, 204, 113, 120),
            "type_fg": QColor(8, 28, 18),
        },
        "TIMER": {
            "row_bg": QColor(160, 120, 220, 35),
            "type_bg": QColor(160, 120, 220, 100),
            "type_fg": QColor(242, 235, 255),
        },
        "TRIGGER": {
            "row_bg": QColor(0, 180, 180, 35),
            "type_bg": QColor(0, 180, 180, 100),
            "type_fg": QColor(230, 255, 255),
        },
        "HOME": {
            "row_bg": QColor(220, 100, 100, 35),
            "type_bg": QColor(220, 100, 100, 110),
            "type_fg": QColor(255, 240, 240),
        },
    }
    
    # Column definitions
    COLUMNS = [
        "Idx", "Motion Type", "X", "Y", "Z", "RX", "RY", "RZ", 
        "Speed", "WeldOn", "Power", "WireFeed", "Weaving",
        "Amp(mm)", "Freq(Hz)", "Dwell(s)",
        "Timer", "Trigger", "Comment"
    ]
    
    # Column indices for easy reference
    COL_IDX = 0
    COL_TYPE = 1
    COL_X = 2
    COL_Y = 3
    COL_Z = 4
    COL_RX = 5
    COL_RY = 6
    COL_RZ = 7
    COL_SPEED = 8
    COL_WELD_ON = 9
    COL_POWER = 10
    COL_WIRE_FEED = 11
    COL_WEAVING = 12
    COL_WEAVE_AMP = 13
    COL_WEAVE_FREQ = 14
    COL_WEAVE_DWELL = 15
    COL_TIMER = 16
    COL_TRIGGER = 17
    COL_COMMENT = 18
    
    def __init__(self, program_rows: Optional[List[ProgramRow]] = None):
        super().__init__()
        self._program_rows = program_rows or []
    
    def rowCount(self, parent: QModelIndex = QModelIndex()) -> int:
        """Return number of rows"""
        return len(self._program_rows)
    
    def columnCount(self, parent: QModelIndex = QModelIndex()) -> int:
        """Return number of columns"""
        return len(self.COLUMNS)
    
    def data(self, index: QModelIndex, role: int = Qt.ItemDataRole.DisplayRole) -> Any:
        """Get data for display or editing"""
        if not index.isValid() or index.row() >= len(self._program_rows):
            return QVariant()
        
        row_data = self._program_rows[index.row()]
        column = index.column()
        
        if role in (Qt.ItemDataRole.DisplayRole, Qt.ItemDataRole.EditRole):
            return self._get_column_data(row_data, column)
        if role == Qt.ItemDataRole.BackgroundRole:
            return self._background_brush(row_data.type, column)
        if role == Qt.ItemDataRole.ForegroundRole:
            return self._foreground_brush(row_data.type, column)
        if role == Qt.ItemDataRole.FontRole and column == self.COL_TYPE:
            font = QFont()
            font.setBold(True)
            return font
        if role == Qt.ItemDataRole.TextAlignmentRole and column == self.COL_TYPE:
            return int(Qt.AlignmentFlag.AlignCenter)
        
        return QVariant()

    def _background_brush(self, motion_type: str, column: int) -> Any:
        style = self.MOTION_TYPE_COLORS.get(str(motion_type).upper())
        if not style:
            return QVariant()
        color = style["type_bg"] if column == self.COL_TYPE else style["row_bg"]
        return QBrush(color)

    def _foreground_brush(self, motion_type: str, column: int) -> Any:
        style = self.MOTION_TYPE_COLORS.get(str(motion_type).upper())
        if not style or column != self.COL_TYPE:
            return QVariant()
        return QBrush(style["type_fg"])
    
    def _get_column_data(self, row_data: ProgramRow, column: int) -> Any:
        """Extract specific column data from ProgramRow"""
        if column == self.COL_IDX:
            return row_data.idx
        elif column == self.COL_TYPE:
            return row_data.type
        elif column == self.COL_X:
            return row_data.pose.x if row_data.pose else 0.0
        elif column == self.COL_Y:
            return row_data.pose.y if row_data.pose else 0.0
        elif column == self.COL_Z:
            return row_data.pose.z if row_data.pose else 0.0
        elif column == self.COL_RX:
            return row_data.pose.rx if row_data.pose else 0.0
        elif column == self.COL_RY:
            return row_data.pose.ry if row_data.pose else 0.0
        elif column == self.COL_RZ:
            return row_data.pose.rz if row_data.pose else 0.0
        elif column == self.COL_SPEED:
            return row_data.speed
        elif column == self.COL_WELD_ON:
            return row_data.weld.on if row_data.weld else False
        elif column == self.COL_POWER:
            return row_data.weld.power if row_data.weld else 0.0
        elif column == self.COL_WIRE_FEED:
            return row_data.weld.wire_feed if row_data.weld else 0.0
        elif column == self.COL_WEAVING:
            return row_data.weld.weaving_type if row_data.weld else "Linear"
        elif column == self.COL_WEAVE_AMP:
            return row_data.weld.weave_amplitude if row_data.weld else 3.0
        elif column == self.COL_WEAVE_FREQ:
            return row_data.weld.weave_frequency if row_data.weld else 2.5
        elif column == self.COL_WEAVE_DWELL:
            return row_data.weld.weave_dwell if row_data.weld else 0.0
        elif column == self.COL_TIMER:
            return row_data.weld.timer if row_data.weld else 0
        elif column == self.COL_TRIGGER:
            return row_data.weld.sensing_trigger if row_data.weld else "None"
        elif column == self.COL_COMMENT:
            return row_data.comment
        
        return ""
    
    def setData(self, index: QModelIndex, value: Any, role: int = Qt.ItemDataRole.EditRole) -> bool:
        """Set data when user edits a cell"""
        if not index.isValid() or index.row() >= len(self._program_rows):
            return False
        
        if role != Qt.ItemDataRole.EditRole:
            return False
        
        row_data = self._program_rows[index.row()]
        column = index.column()
        
        try:
            success = self._set_column_data(row_data, column, value)
            if success:
                left = self.index(index.row(), 0)
                right = self.index(index.row(), self.columnCount() - 1)
                self.dataChanged.emit(
                    left,
                    right,
                    [
                        Qt.ItemDataRole.DisplayRole,
                        Qt.ItemDataRole.EditRole,
                        Qt.ItemDataRole.BackgroundRole,
                        Qt.ItemDataRole.ForegroundRole,
                        Qt.ItemDataRole.FontRole,
                    ],
                )
            return success
        except (ValueError, TypeError) as e:
            print(f"Invalid data for column {column}: {e}")
            return False
    
    def _set_column_data(self, row_data: ProgramRow, column: int, value: Any) -> bool:
        """Set specific column data in ProgramRow with validation"""
        try:
            if column == self.COL_IDX:
                row_data.idx = int(value)
            elif column == self.COL_TYPE:
                row_data.type = str(value)
            elif column == self.COL_X:
                if not row_data.pose:
                    row_data.pose = Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                row_data.pose.x = float(value)
            elif column == self.COL_Y:
                if not row_data.pose:
                    row_data.pose = Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                row_data.pose.y = float(value)
            elif column == self.COL_Z:
                if not row_data.pose:
                    row_data.pose = Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                row_data.pose.z = float(value)
            elif column == self.COL_RX:
                if not row_data.pose:
                    row_data.pose = Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                row_data.pose.rx = float(value)
            elif column == self.COL_RY:
                if not row_data.pose:
                    row_data.pose = Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                row_data.pose.ry = float(value)
            elif column == self.COL_RZ:
                if not row_data.pose:
                    row_data.pose = Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                row_data.pose.rz = float(value)
            elif column == self.COL_SPEED:
                row_data.speed = float(value)
            elif column == self.COL_WELD_ON:
                if not row_data.weld:
                    row_data.weld = WeldParams(False, 0.0, 0.0, 0.5, 1.0, "Linear", 0, "None")
                # Convert string representations to boolean
                if isinstance(value, str):
                    row_data.weld.on = value.lower() in ('true', '1', 'yes', 'on')
                else:
                    row_data.weld.on = bool(value)
            elif column == self.COL_POWER:
                if not row_data.weld:
                    row_data.weld = WeldParams(False, 0.0, 0.0, 0.5, 1.0, "Linear", 0, "None")
                power = float(value)
                # Validate power range (0-100)
                if 0.0 <= power <= 100.0:
                    row_data.weld.power = power
                else:
                    raise ValueError(f"Power must be between 0-100, got {power}")
            elif column == self.COL_WIRE_FEED:
                if not row_data.weld:
                    row_data.weld = WeldParams(False, 0.0, 0.0, 0.5, 1.0, "Linear", 0, "None")
                wire_feed = float(value)
                # Validate wire feed (positive value)
                if wire_feed >= 0.0:
                    row_data.weld.wire_feed = wire_feed
                else:
                    raise ValueError(f"Wire feed must be positive, got {wire_feed}")
            elif column == self.COL_WEAVE_AMP:
                if not row_data.weld:
                    row_data.weld = WeldParams(False, 0.0, 0.0, 0.5, 1.0, "Linear", 0, "None")
                amp = float(value)
                if 0.0 <= amp <= 50.0:
                    row_data.weld.weave_amplitude = amp
                else:
                    raise ValueError(f"Amplitude must be 0-50 mm, got {amp}")
            elif column == self.COL_WEAVE_FREQ:
                if not row_data.weld:
                    row_data.weld = WeldParams(False, 0.0, 0.0, 0.5, 1.0, "Linear", 0, "None")
                freq = float(value)
                if 0.1 <= freq <= 20.0:
                    row_data.weld.weave_frequency = freq
                else:
                    raise ValueError(f"Frequency must be 0.1-20 Hz, got {freq}")
            elif column == self.COL_WEAVE_DWELL:
                if not row_data.weld:
                    row_data.weld = WeldParams(False, 0.0, 0.0, 0.5, 1.0, "Linear", 0, "None")
                dwell = float(value)
                if 0.0 <= dwell <= 5.0:
                    row_data.weld.weave_dwell = dwell
                else:
                    raise ValueError(f"Dwell must be 0-5 s, got {dwell}")
            elif column == self.COL_COMMENT:
                row_data.comment = str(value)
            else:
                return False
            
            return True
            
        except (ValueError, TypeError):
            return False
    
    def headerData(self, section: int, orientation: Qt.Orientation, 
                   role: int = Qt.ItemDataRole.DisplayRole) -> Any:
        """Return header data"""
        if role == Qt.ItemDataRole.DisplayRole:
            if orientation == Qt.Orientation.Horizontal:
                if 0 <= section < len(self.COLUMNS):
                    return self.COLUMNS[section]
            elif orientation == Qt.Orientation.Vertical:
                return str(section + 1)
        
        return QVariant()
    
    def flags(self, index: QModelIndex) -> Qt.ItemFlag:
        """Return item flags (editable, selectable, etc.)"""
        if not index.isValid():
            return Qt.ItemFlag.NoItemFlags
        
        return (Qt.ItemFlag.ItemIsEnabled | 
                Qt.ItemFlag.ItemIsSelectable | 
                Qt.ItemFlag.ItemIsEditable)
    
    def get_program_rows(self) -> List[ProgramRow]:
        """Get the current program rows"""
        return self._program_rows
    
    @property
    def program_data(self) -> List[ProgramRow]:
        """Property accessor for program data"""
        return self._program_rows
    
    def set_program_rows(self, program_rows: List[ProgramRow]):
        """Set new program rows and refresh the view"""
        self.beginResetModel()
        self._program_rows = program_rows
        self.endResetModel()
    
    def set_program_data(self, program_rows: List[ProgramRow]):
        """Alias for set_program_rows for consistency"""
        self.set_program_rows(program_rows)
    
    def add_row(self, program_row: ProgramRow):
        """Add a new program row"""
        row_count = len(self._program_rows)
        self.beginInsertRows(QModelIndex(), row_count, row_count)
        self._program_rows.append(program_row)
        self.endInsertRows()
    
    def remove_row(self, row_index: int) -> bool:
        """Remove a program row"""
        if 0 <= row_index < len(self._program_rows):
            self.beginRemoveRows(QModelIndex(), row_index, row_index)
            del self._program_rows[row_index]
            self.endRemoveRows()
            return True
        return False
