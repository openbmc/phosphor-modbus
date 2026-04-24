#!/usr/bin/env python3
"""Validate device profile JSON files against the schema."""

import json
import sys

import jsonschema


def main():
    schema_path = sys.argv[1]
    profile_paths = sys.argv[2:]

    with open(schema_path) as f:
        schema = json.load(f)

    errors = 0
    for path in profile_paths:
        with open(path) as f:
            profile = json.load(f)
        try:
            jsonschema.validate(profile, schema)
        except jsonschema.ValidationError as e:
            print(f"FAIL: {path}: {e.message}", file=sys.stderr)
            errors += 1

    if errors:
        sys.exit(1)


if __name__ == "__main__":
    main()
