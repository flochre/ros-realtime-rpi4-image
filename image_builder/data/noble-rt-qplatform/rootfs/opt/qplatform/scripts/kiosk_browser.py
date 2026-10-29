#!/usr/bin/env python3
"""
Minimal zero-chrome Wayland WebKit2GTK Kiosk Browser for QPlatform (Cage).
Runs natively on Wayland inside `cage` without X11, desktop environment, or snapd.
"""
import os
import sys
import signal
import gi

gi.require_version("Gtk", "3.0")
gi.require_version("Gdk", "3.0")
gi.require_version("WebKit2", "4.1")
from gi.repository import Gtk, Gdk, GLib, WebKit2


class KioskBrowser(Gtk.Window):
    def __init__(self, url: str, hide_cursor: bool = False):
        super().__init__(title="QPlatform Kiosk")
        self.url = url
        self.hide_cursor = hide_cursor

        self.set_decorated(False)
        self.fullscreen()
        self.connect("destroy", Gtk.main_quit)

        # Configure WebKit settings for embedded kiosk use
        settings = WebKit2.Settings()
        settings.set_enable_javascript(True)
        settings.set_enable_webgl(True)
        settings.set_enable_webaudio(True)
        settings.set_enable_accelerated_2d_canvas(True)
        settings.set_enable_smooth_scrolling(True)
        settings.set_enable_developer_extras(False)
        settings.set_enable_write_console_messages_to_stdout(True)
        settings.set_media_playback_requires_user_gesture(False)

        # Disable context menu (right-click / long-press menu on touchscreens)
        self.webview = WebKit2.WebView.new_with_settings(settings)
        self.webview.connect("context-menu", lambda *args: True)

        # Automatically retry loading if the local web server isn't up yet
        self.webview.connect("load-failed", self.on_load_failed)

        # Dark background while loading
        rgba = Gdk.RGBA()
        rgba.parse("#0f172a")
        self.webview.set_background_color(rgba)

        self.add(self.webview)
        self.show_all()

        if self.hide_cursor:
            self.connect("realize", self._hide_pointer)

        self.webview.load_uri(self.url)

    def _hide_pointer(self, widget):
        window = widget.get_window()
        if window:
            display = window.get_display()
            cursor = Gdk.Cursor.new_for_display(display, Gdk.CursorType.BLANK_CURSOR)
            window.set_cursor(cursor)

    def on_load_failed(self, webview, load_event, failing_uri, error):
        # Retry after 2 seconds if target web server is still starting up
        GLib.timeout_add_seconds(2, self._retry_load)
        return True

    def _retry_load(self):
        self.webview.load_uri(self.url)
        return False


def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)

    url = sys.argv[1] if len(sys.argv) > 1 else os.environ.get(
        "KIOSK_URL", "file:///opt/qplatform/webui/index.html"
    )
    hide_cursor = os.environ.get("KIOSK_HIDE_CURSOR", "0") == "1"

    KioskBrowser(url, hide_cursor=hide_cursor)
    Gtk.main()


if __name__ == "__main__":
    main()
