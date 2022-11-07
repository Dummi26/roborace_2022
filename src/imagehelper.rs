use image::{DynamicImage, io::Reader as ImageReader};

pub fn read_image_from_file<P>(path: P) -> Result<DynamicImage, ReadImageFromFileError> where P: AsRef<std::path::Path> {
    match ImageReader::open(path) {
        Ok(reader) => match reader.decode() {
            Ok(img) => Ok(img),
            Err(e) => Err(ReadImageFromFileError::FailedToDecodeImage(e)),
        },
        Err(e) => Err(ReadImageFromFileError::FailedToOpenFile(e)),
    }
}
pub enum ReadImageFromFileError {
    FailedToOpenFile(std::io::Error),
    FailedToDecodeImage(image::ImageError),
}

pub enum PotImage {
    Path(std::path::PathBuf),
    Img(DynamicImage),
}
impl PotImage {
    pub fn new<P>(path: P) -> Self where P: Into<std::path::PathBuf> {
        Self::Path(path.into())
    }
    /// Loads the image from disk (if necessary).
    pub fn load(&mut self) -> Result<bool, ReadImageFromFileError> {
        match self {
            Self::Path(path) => {
                *self = Self::Img(read_image_from_file(path)?);
                Ok(true)
            }
            Self::Img(_) => Ok(false),
        }
    }

    pub fn get(&mut self) -> Option<&mut DynamicImage> {
        self.load();
        if let Self::Img(img) = self {
            Some(img)
        } else {
            None
        }
    }
}