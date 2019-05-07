//
// debugger.rs
// Copyright (C) 2019 Emiliano Firmino <emiliano.firmino@gmail.com>
// Distributed under terms of the MIT license.
//
use std::process;
use sfml::graphics::{Color, RenderTarget, RenderWindow};
use sfml::window::{Event, Key, Style};

pub struct Debugger {
    window: RenderWindow,
}

impl Debugger {
    pub fn new() -> Debugger {
        Debugger {
            window: RenderWindow::new((800, 600), "Debugger", Style::CLOSE, &Default::default())
        }
    }

    pub fn refresh(&mut self) {
        while let Some(event) = self.window.poll_event() {
            match event {
                Event::Closed | Event::KeyPressed { code: Key::Escape, .. } => process::exit(0),
                _ => {}
            }
        }
        self.window.clear(&Color::WHITE);
        self.window.display();
    }
}
