use std::{path::PathBuf, io::{self, Read}};

use image::{DynamicImage, imageops::FilterType};

pub fn load_img(path: std::path::PathBuf) -> Option<DynamicImage> {
    match std::fs::File::open(&path) {
        Ok(mut file) => {
            let mut buf = Vec::new();
            if let io::Result::Err(err) = file.read_to_end(&mut buf) { eprintln!("While reading bytes from file, an error was encountered: {err}",); return None; };
            match image::io::Reader::new(io::Cursor::new(buf)).with_guessed_format() {
                Ok(img) => {
                    match img.decode() {
                        Ok(img) => Some(img),
                        Err(err) => {
                            eprintln!("Could not load image: {err}");
                            None
                        },
                    }
                },
                Err(err) => {
                    eprintln!("Could not guess image format: {err}");
                    None
                },
            }
        },
        Err(err) => {
            eprintln!("Could not open file at '{:?}': {:?}", path, err);
            None
        },
    }
}