use std::collections::HashMap;

#[derive(Debug)]
pub struct RosMsg {
    pub module: String, // e.g. std_msgs
    pub prefix: String, // e.g. "msg" or "srv"
    pub name: String, // e.g. "String"
}

pub fn parse_msgs(msgs: &str) -> Vec<RosMsg> {
    let v: Vec<Vec<&str>> = msgs.lines().map(|l| l.split("/").into_iter().take(3).collect()).collect();
    v.iter().filter(|v|v.len() == 3).
        map(|v| RosMsg { module: v[0].into(), prefix: v[1].into(), name: v[2].into()}).collect()

}

pub fn as_map(included_msgs: &[RosMsg]) -> HashMap<&str, HashMap<&str, Vec<&str>>> {
    let mut msgs = HashMap::new();
    for msg in included_msgs {
        msgs.entry(msg.module.as_str()).or_insert(HashMap::new()).entry(msg.prefix.as_str()).
            or_insert(Vec::new()).push(msg.name.as_str());
    }
    msgs
}

use std::io::{self, Read};
use std::fs::File;

pub fn read_file(filename: &str) -> io::Result<String> {
    let mut file = File::open(filename)?;
    let mut s = String::new();
    file.read_to_string(&mut s)?;
    Ok(s)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_msgs() -> () {
        let msgs = "
std_msgs/msg/Bool
x/y
std_msgs/msg/String
";
        let parsed = parse_msgs(msgs);
        assert_eq!(parsed[0].module, "std_msgs");
        assert_eq!(parsed[0].prefix, "msg");
        assert_eq!(parsed[0].name, "Bool");
        assert_eq!(parsed[1].module, "std_msgs");
        assert_eq!(parsed[1].prefix, "msg");
        assert_eq!(parsed[1].name, "String");
    }

    #[test]
    fn test_as_map() -> () {
        let msgs = "
std_msgs/msg/Bool
x/y
std_msgs/msg/String
";
        let parsed = parse_msgs(msgs);
        let map = as_map(&parsed);

        assert_eq!(map.get("std_msgs").unwrap().get("msg").unwrap()[0], "Bool");
        assert_eq!(map.get("std_msgs").unwrap().get("msg").unwrap()[1], "String");
        

    }    
        

}

