use r2r;
use failure::Error;


fn main() -> Result<(), Error> {
    let mut clock = r2r::RosClock::create()?;
    let now = clock.get_now()?;
    let time = r2r::RosClock::to_builtin_time(&now);
    println!("time: {:?}", time);
    Ok(())
}
