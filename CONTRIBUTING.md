# 코딩 컨벤션

## 브랜치 규칙

브랜치는 관련 이슈를 확인한 뒤 `<유형>/<이슈 번호>-<요약>` 형식으로 만듭니다.

| 유형 | 예시 |
| --- | --- |
| `feature` | `feature/123-search` |
| `fix` | `fix/124-empty-input` |
| `task` | `task/125-coding-conventions` |

- `<이슈 번호>`는 `#` 없이 숫자로만 작성합니다.
- `<요약>`은 핵심을 나타내는 짧은 명사형으로, `kebab-case`로 작성합니다.

## PR 및 커밋 규칙

PR 제목이나 커밋 메시지는 `<유형>: <요약>` 형식으로 만듭니다.

| 유형 | 용도 |
| --- | --- |
| `feat` | 기능 추가 |
| `fix` | 버그 수정 |
| `docs` | 문서 변경 |
| `style` | 코드 동작에 영향을 주지 않는 서식 변경 |
| `refactor` | 기능 변경 없는 코드 개선 |
| `test` | 테스트 추가 또는 수정 |
| `chore` | 빌드, 설정 등 기타 작업 |

- `<요약>`은 작업 내용을 간결하게 나타내고, 끝에 마침표를 붙이지 않습니다.
- PR은 **Squash and merge** 방식으로만 병합합니다. (2026-10-03부터 적용)

## 설계 원칙

객체지향 설계 시 SOLID를 준수합니다.

- **단일 책임 원칙 (SRP)**: 클래스와 함수는 하나의 책임만 갖도록 합니다.
- **개방-폐쇄 원칙 (OCP)**: 기존 코드를 수정하는 대신 확장을 통해 기능을 추가할 수 있도록 합니다.
- **리스코프 치환 원칙 (LSP)**: 하위 타입은 상위 타입을 사용하는 코드에서 대체 가능해야 합니다.
- **인터페이스 분리 원칙 (ISP)**: 사용하지 않는 기능에 의존하지 않도록 인터페이스를 작고 구체적으로 설계합니다.
- **의존관계 역전 원칙 (DIP)**: 구체적인 구현보다 추상화에 의존하도록 합니다.

## C++ 네이밍 규칙

| 대상 | 규칙 |
| --- | --- |
| 디렉터리 | `snake_case` |
| 파일 | `snake_case` |
| namespace | `snake_case` |
| 클래스 | `PascalCase` |
| 구조체 | `PascalCase` |
| enum 타입 | `PascalCase` |
| enum 값 | `UPPER_SNAKE_CASE` |
| 함수 | `camelCase` |
| 클래스의 메소드 | `camelCase` |
| 클래스의 private 멤버 변수 | `_camelCase` |
| 구조체의 public 멤버 변수 | `camelCase` |
| 매개변수 | `camelCase` |
| 지역 변수 | `camelCase` |
| 상수 | `UPPER_SNAKE_CASE` |

- `ST_*` 형식의 구조체는 예외적으로 이 규칙을 따르지 않습니다.
- 좌표계나 단위는 필요한 경우에만 이름에 `_ant_deg`와 같이 덧붙입니다.

## C++ 코드 포맷

```cpp
#include <algorithm>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

namespace print
{
class NumberPrinter
{
public:
    void print(
        const std::vector<int>& values,
        const std::string& title,
        int minimumValue
    ) const {
        if (values.empty())
        {
            std::cout << "No numbers" << '\n';
            return;
        }

        std::cout << title << '\n';
        for (std::size_t index = 0; index < values.size(); ++index)
        {
            int value = std::max(values[index], minimumValue);
            int remainder = value % 2;

            switch (remainder)
            {
            case 0:
                std::cout << "Even: ";
                break;

            default:
                std::cout << "Odd: ";
                break;
            }

            std::cout << value << _separator;
        }
    }

private:
    char _separator = '\n';
};
}

int main()
{
    const int INITIAL_VALUES[] =
    {
        1,
        2,
        3
    };

    const std::vector<int> values(std::begin(INITIAL_VALUES), std::end(INITIAL_VALUES));
    print::NumberPrinter printer;
    printer.print(
        values,
        "Example numbers",
        0);

    return 0;
}
```
