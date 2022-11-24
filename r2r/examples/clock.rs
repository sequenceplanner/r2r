fn main() -> Result<(), Box<dyn std::error::Error>> {
    {
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime)?;
        let now = clock.get_now()?;
        let time = r2r::Clock::to_builtin_time(&now);
        println!("rostime: {:?}", time);
    }
    {
        let mut clock = r2r::Clock::create(r2r::ClockType::SystemTime)?;
        let now = clock.get_now()?;
        let time = r2r::Clock::to_builtin_time(&now);
        println!("systemtime: {:?}", time);
    }
    {
        let mut clock = r2r::Clock::create(r2r::ClockType::SteadyTime)?;
        let now = clock.get_now()?;
        let time = r2r::Clock::to_builtin_time(&now);
        println!("steadytime: {:?}", time);
    }
    Ok(())
}
