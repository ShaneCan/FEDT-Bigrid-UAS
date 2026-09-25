# How to Contribute

Thank you for your interest in contributing to UniAgent.

Whether you're fixing bugs, improving documentation, or developing new features, we greatly appreciate your help.

## Getting Started

Before contributing, please check out the following resources:
- Use the [🐞 Bug Reports & Feature Requests](https://github.com/UniAgent-Platform/bigrid-provider-service/issues) to report problems or suggest improvements. Be sure to search existing issues before submitting a new one.

## Submitting Changes

To contribute code, submit a *GitHub Pull Request* to this repository with a clear explanation of your changes.

> Tip: Smaller, focused PRs are easier to review and merge.

Make sure to:
- Include relevant unit tests and/or examples.
- Follow our coding conventions (see below).
- Ensure each commit is atomic (one logical change per commit).
- Provide a descriptive commit message:

```text
Add support for XYZ bigraph reaction

This adds support for a new kind of reaction rule, with associated
tests and documentation. It extends the matching subsystem to
support partial embedding scenarios.
```

## Code Style & Conventions

We aim for readable, maintainable code.
When in doubt, prioritize clarity.

Please follow these basic conventions:
- Indentation: Use four spaces (no tabs).
- Spacing:
    - Place spaces after list items and method parameters: [1, 2, 3] not [1,2,3].
    - Put spaces around operators: x += 1, not x+=1.
    - Add whitespace around map arrows and binary operators.
- Comments: Favor clear, precise comments that explain why something is done, especially in formal or algorithmic parts.
- Design: Aim for modular, testable code with minimal side effects.

## Tests & Coverage

- If you fix a bug, add a regression test.
- If you add functionality, include unit tests that illustrate typical usage.
- We use [JUnit Jupiter](https://junit.org/) for Jupiter based testing. We expect all tests to pass before merging.

Thanks!