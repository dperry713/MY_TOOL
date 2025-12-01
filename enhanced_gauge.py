"""
Enhanced CircularGauge with tick marks, segmented color arcs, and compact mode.
"""

import math
from PyQt6.QtWidgets import QWidget
from PyQt6.QtCore import Qt, QRect, QPoint, QPointF
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QFont, QLinearGradient


class CircularGauge(QWidget):
    def __init__(self, min_value=0, max_value=100, parent=None, compact=False, title="", yellow_threshold=0.6, red_threshold=0.85):
        super().__init__(parent)
        self.min_value = min_value
        self.max_value = max_value
        self.current_value = min_value
        self.start_angle = 225  # Starting angle in degrees
        self.span_angle = 270   # Span in degrees
        self.compact = compact  # If True, show a small thin gauge
        self.title = title  # Title for the gauge
        self.yellow_threshold = yellow_threshold  # Fraction [0..1] where yellow starts
        self.red_threshold = red_threshold        # Fraction [0..1] where red starts
        if compact:
            self.setMinimumSize(140, 140)
            self.setMaximumSize(160, 160)
        else:
            self.setMinimumSize(280, 280)  # Larger for main gauges

    def set_value(self, value):
        val = max(self.min_value, min(self.max_value, value))
        self.current_value = val
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        side = min(self.width(), self.height())
        margin = 6 if self.compact else 12
        outer_rect = QRect(margin, margin, side - 2*margin, side - 2*margin)
        center = outer_rect.center()
        radius = (outer_rect.width() / 2)

        # Dark background circle (dark gray with subtle gradient)
        gradient_bg = QLinearGradient(QPointF(outer_rect.topLeft()), QPointF(outer_rect.bottomRight()))
        gradient_bg.setColorAt(0, QColor(35, 35, 35))
        gradient_bg.setColorAt(1, QColor(20, 20, 20))
        painter.setBrush(gradient_bg)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(outer_rect)

        progress = (self.current_value - self.min_value) / (self.max_value - self.min_value)
        progress = max(0.0, min(1.0, progress))
        
        if self.compact:
            # Compact gauge: thin outer arc
            arc_width = 8
            arc_rect = QRect(outer_rect.x() + 8, outer_rect.y() + 8, 
                            outer_rect.width() - 16, outer_rect.height() - 16)
            # Draw background arc
            pen = QPen(QColor(60, 60, 60), arc_width, Qt.PenStyle.SolidLine)
            painter.setPen(pen)
            painter.drawArc(arc_rect, self.start_angle * 16, -self.span_angle * 16)
            # Draw progress arc with color based on progress
            angle = self.span_angle * progress
            if progress < self.yellow_threshold:
                arc_color = QColor(0, 200, 150)  # Teal
            elif progress < self.red_threshold:
                arc_color = QColor(255, 200, 0)  # Yellow
            else:
                arc_color = QColor(255, 80, 80)  # Red
            pen_progress = QPen(arc_color, arc_width, Qt.PenStyle.SolidLine)
            painter.setPen(pen_progress)
            painter.drawArc(arc_rect, self.start_angle * 16, -int(angle * 16))
            # Draw value text (small font)
            painter.setPen(QColor(220, 220, 220))
            font = QFont("Segoe UI", 11, QFont.Weight.Bold)
            painter.setFont(font)
            painter.drawText(outer_rect, Qt.AlignmentFlag.AlignCenter, f"{int(self.current_value)}")
            # Draw title below
            if self.title:
                painter.setPen(QColor(180, 180, 180))
                font_title = QFont("Segoe UI", 8)
                painter.setFont(font_title)
                painter.drawText(QRect(outer_rect.x(), outer_rect.bottom() - 16, outer_rect.width(), 14),
                               Qt.AlignmentFlag.AlignCenter, self.title)
        else:
            # Full gauge: background arc with grid/ticks
            arc_width = 18
            arc_rect = QRect(outer_rect.x() + 20, outer_rect.y() + 20,
                            outer_rect.width() - 40, outer_rect.height() - 40)
            
            # Draw numbered tick marks
            num_ticks = 10
            for i in range(num_ticks + 1):
                tick_angle = self.start_angle - (self.span_angle * i / num_ticks)
                is_major = (i % 2 == 0)
                tick_len = 12 if is_major else 6
                tick_width = 2 if is_major else 1
                inner_r = radius - 35 - tick_len
                outer_r = radius - 35
                tick_start_x = center.x() + inner_r * math.cos(math.radians(tick_angle))
                tick_start_y = center.y() - inner_r * math.sin(math.radians(tick_angle))
                tick_end_x = center.x() + outer_r * math.cos(math.radians(tick_angle))
                tick_end_y = center.y() - outer_r * math.sin(math.radians(tick_angle))
                pen_tick = QPen(QColor(150, 150, 150), tick_width, Qt.PenStyle.SolidLine)
                painter.setPen(pen_tick)
                painter.drawLine(int(tick_start_x), int(tick_start_y), int(tick_end_x), int(tick_end_y))
                # Draw tick labels for major ticks
                if is_major:
                    label_val = int(self.min_value + (self.max_value - self.min_value) * i / num_ticks)
                    label_r = radius - 20
                    label_x = center.x() + label_r * math.cos(math.radians(tick_angle))
                    label_y = center.y() - label_r * math.sin(math.radians(tick_angle))
                    painter.setPen(QColor(200, 200, 200))
                    font_tick = QFont("Segoe UI", 8)
                    painter.setFont(font_tick)
                    painter.drawText(int(label_x - 12), int(label_y - 8), 24, 16,
                                   Qt.AlignmentFlag.AlignCenter, str(label_val))
            
            # Draw background arc
            pen = QPen(QColor(60, 60, 60), arc_width, Qt.PenStyle.SolidLine)
            painter.setPen(pen)
            painter.drawArc(arc_rect, self.start_angle * 16, -self.span_angle * 16)
            
            # Draw segmented progress arc (color-coded)
            angle_for_val = self.span_angle * progress
            # Green zone: 0 to yellow_threshold
            green_angle = self.span_angle * self.yellow_threshold
            pen_green = QPen(QColor(0, 200, 150), arc_width, Qt.PenStyle.SolidLine)
            painter.setPen(pen_green)
            painter.drawArc(arc_rect, self.start_angle * 16, -int(min(angle_for_val, green_angle) * 16))
            # Yellow zone: yellow_threshold to red_threshold
            if angle_for_val > green_angle:
                yellow_angle = min(angle_for_val - green_angle, 
                                 self.span_angle * (self.red_threshold - self.yellow_threshold))
                pen_yellow = QPen(QColor(255, 200, 0), arc_width, Qt.PenStyle.SolidLine)
                painter.setPen(pen_yellow)
                start_angle_yellow = self.start_angle - green_angle
                painter.drawArc(arc_rect, int(start_angle_yellow * 16), -int(yellow_angle * 16))
            # Red zone: red_threshold and beyond
            if angle_for_val > self.span_angle * self.red_threshold:
                red_angle = angle_for_val - self.span_angle * self.red_threshold
                pen_red = QPen(QColor(255, 80, 80), arc_width, Qt.PenStyle.SolidLine)
                painter.setPen(pen_red)
                start_angle_red = self.start_angle - self.span_angle * self.red_threshold
                painter.drawArc(arc_rect, int(start_angle_red * 16), -int(red_angle * 16))
            
            # Draw needle
            needle_angle = self.start_angle - angle_for_val
            needle_length = radius - 45
            dx = math.cos(math.radians(needle_angle)) * needle_length
            dy = -math.sin(math.radians(needle_angle)) * needle_length
            pen_needle = QPen(QColor(240, 240, 240), 4, Qt.PenStyle.SolidLine)
            painter.setPen(pen_needle)
            painter.drawLine(center, QPoint(int(center.x() + dx), int(center.y() + dy)))
            # Center cap
            painter.setBrush(QColor(40, 40, 40))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawEllipse(QRect(center.x() - 7, center.y() - 7, 14, 14))
            
            # Draw value text (large, center)
            painter.setPen(QColor(220, 220, 220))
            font = QFont("Segoe UI", 24, QFont.Weight.Bold)
            painter.setFont(font)
            value_rect = QRect(center.x() - 50, center.y() - 20, 100, 40)
            painter.drawText(value_rect, Qt.AlignmentFlag.AlignCenter, f"{int(self.current_value)}")
            # Draw title
            if self.title:
                painter.setPen(QColor(180, 180, 180))
                font_title = QFont("Segoe UI", 10)
                painter.setFont(font_title)
                painter.drawText(QRect(center.x() - 60, center.y() + 25, 120, 20),
                               Qt.AlignmentFlag.AlignCenter, self.title)
