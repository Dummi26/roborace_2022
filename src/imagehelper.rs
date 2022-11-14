use image::{DynamicImage, io::Reader as ImageReader};

// Screen: 178 x 128

pub struct ImageKeeper {
    pub image_err_missing_ultraschallsensor: PotImage,
}
impl ImageKeeper {
    pub fn new() -> Self { Self {
        image_err_missing_ultraschallsensor:
            PotImage::new("/home/robot/school/assets/ultraschallsensor_missing.png"),
    } }
    pub fn get_err_missing_dev(&mut self, dev: &crate::roboter::Device) -> Option<&mut DynamicImage> {
        match dev {
            crate::roboter::Device::LargeMotor1 => self.image_err_missing_ultraschallsensor.get(),
            crate::roboter::Device::LargeMotor2 => self.image_err_missing_ultraschallsensor.get(),
            crate::roboter::Device::MediumMotor => self.image_err_missing_ultraschallsensor.get(),
            crate::roboter::Device::ColorSensor => self.image_err_missing_ultraschallsensor.get(),
            crate::roboter::Device::GyroSensor => self.image_err_missing_ultraschallsensor.get(),
            crate::roboter::Device::TouchSensor => self.image_err_missing_ultraschallsensor.get(),
            crate::roboter::Device::UltrasonicSensor => self.image_err_missing_ultraschallsensor.get(),
        }
    }
}

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
        _ = self.load();
        if let Self::Img(img) = self {
            Some(img)
        } else {
            None
        }
    }
}