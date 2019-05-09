//
// debugger.rs
// Copyright (C) 2019 Emiliano Firmino <emiliano.firmino@gmail.com>
// Distributed under terms of the MIT license.
//
use std::process;
use sfml::graphics::{Color, Font, RenderTarget, RenderWindow, Text};
use sfml::window::{Event, Key, Style};

use crate::cpu::Cpu;
use crate::mmu::Mmu;

pub struct Debugger {
    font: Font,
    window: RenderWindow,
    pub step_mode: bool,
    pub step_once: bool,
}

impl Debugger {
    pub fn new() -> Debugger {
        Debugger {
            font: Font::from_file("res/DejaVuSansMono.ttf").unwrap(),
            step_mode: false,
            step_once: false,
            window: RenderWindow::new((1200, 960), "Debugger", Style::CLOSE, &Default::default())
        }
    }

    pub fn refresh(&mut self, cpu: &Cpu, mmu: &Mmu) {
        while let Some(event) = self.window.poll_event() {
            match event {
                Event::Closed  => process::exit(0),
                Event::KeyPressed { code: Key::Escape, .. } => self.step_mode = false,
                Event::KeyPressed { code: Key::Return, .. } => {
                    self.step_mode = true;
                    self.step_once = true;
                },
                _ => {}
            }
        }
        let mut text = Text::new(&format!("{:?}\n{}", cpu, cpu.disasm(&mmu, 35)), &self.font, 20);
        text.set_fill_color(&Color::BLACK);

        self.window.clear(&Color::WHITE);
        self.window.draw(&text);
        self.window.display();
    }
}
