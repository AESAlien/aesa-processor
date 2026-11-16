# GitHub Actions 사용 안내

저장소는 GitHub Actions로 코드 검증, 배포 패키지 생성, 종료된 Pull Request 승인 작업을 수행합니다.
각 작업은 독립된 워크플로로 관리합니다.

| 워크플로 | 목적 | 실행 방식 |
| --- | --- | --- |
| [CI](ci.yml) | 변경 사항 빌드 및 테스트 | `main` push 또는 `main` 대상 Pull Request |
| [CD](cd.yml) | 검증된 빌드 패키징 및 Release 게시 | `main` push, 버전 태그 push 또는 수동 실행 |
| [PR 수동 승인](manual-approve.yml) | 지정한 종료 PR에 승인 리뷰 제출 | GitHub Actions에서 PR 번호를 입력해 수동 실행 |

## CI — 코드 검증

CI는 `main` 브랜치에 들어오는 변경을 Debug 설정으로 빌드하고 테스트합니다.

### 언제 실행되는가

- `main` 브랜치에 커밋을 push할 때 실행합니다.
- `main`을 대상으로 Pull Request를 열거나 다시 열고, 커밋을 갱신할 때 실행합니다.

### 무엇을 확인하는가

Ubuntu 러너에서 저장소를 체크아웃하고 `libgtest-dev`를 설치합니다. CMake Debug 빌드를
구성한 뒤 모든 타깃을 병렬로 빌드하고 CTest 전체 테스트를 실행합니다. 테스트가 실패하면
실패 상세 로그를 출력합니다.

CI는 `contents: read` 권한만 사용합니다. 패키지를 만들거나 Release를 게시하지 않습니다.
실행 결과는 GitHub Actions의 CI 실행 내역과 Pull Request의 Checks에서 확인할 수 있습니다.

## CD — 배포 패키지

CD는 Release 설정으로 빌드와 테스트를 수행하고, 성공한 경우 실행 패키지를 만듭니다.
CI와 별도로 동작하며, 배포할 Release 빌드에서 테스트를 다시 실행합니다.

### 실행 방법과 결과

| 실행 조건 | 결과 |
| --- | --- |
| `main`에 push | 패키지를 Actions Artifacts에 30일간 보관 |
| `vMAJOR.MINOR.PATCH` 태그 push | 패키지를 보관하고 GitHub Release에 게시 |
| Actions에서 **CD → Run workflow** 선택 | 선택한 브랜치의 패키지를 수동 생성 |

태그로 배포할 때는 `main`에 포함된 커밋에 아직 사용하지 않은 버전 태그를 붙여 push합니다.

```sh
git tag v1.0.0
git push origin v1.0.0
```

워크플로는 `vMAJOR.MINOR.PATCH` 형식이 아닌 태그를 거부합니다. 태그가 가리키는 커밋도
`main`에 포함되어야 합니다. 빌드와 테스트가 성공하면 패키지를 생성하며, 테스트가 없거나
실패하면 패키징과 Release 게시를 중단합니다.

Release를 공개하기 전에 원격 태그가 현재 빌드한 커밋을 가리키는지 확인합니다. 태그가
삭제되거나 이동했거나 확인할 수 없으면 게시하지 않습니다. 이 확인과 공개는 하나의 원자적
작업이 아니므로, 그 사이의 태그 변경도 막으려면 저장소 규칙에서 릴리스 태그의 수정과
삭제를 차단해야 합니다. 공개 전에 실패해 초안 Release가 남으면 초안을 삭제한 뒤 작업을
다시 실행합니다. 공개된 Release는 덮어쓰지 않으므로 수정 배포에는 새 버전 태그를 사용합니다.

### 패키지 내려받기와 실행

패키지와 `.sha256` 파일은 Actions Artifacts에서 함께 내려받습니다. Artifact ZIP을 풀고
다음 명령으로 체크섬을 확인한 뒤 패키지를 실행합니다. `<커밋 SHA>`는 내려받은 파일명의
실제 커밋 SHA로 바꿉니다.

```sh
sha256sum --check aesa-processor-<커밋 SHA>-linux-x86_64.tar.gz.sha256
tar -xzf aesa-processor-<커밋 SHA>-linux-x86_64.tar.gz
cd aesa-processor-<커밋 SHA>-linux-x86_64
cp .env.example .env
# .env를 운용 환경에 맞게 설정한 뒤 실행합니다.
./adp
```

패키지는 Ubuntu 24.04 x86_64용이며 해당 시스템의 glibc와 libstdc++ 런타임을 사용합니다.
실행 파일, `.env.example`, 프로젝트 `README.md`, 이 문서의 복사본 `CI_CD.md`, 빌드 커밋을
기록한 `COMMIT`, 외부 라이브러리 라이선스를 포함합니다. 실제 `.env`는 포함하지 않습니다.
운용 장비에 설치하고 프로세스를 재시작하는 작업은 운영자가 수행합니다. 이전 버전으로
복구할 때는 기존 프로세스를 종료하고 해당 Release 패키지에 운용 설정을 적용해 실행합니다.

### 권한과 저장소 설정

기본 `GITHUB_TOKEN`을 사용하므로 별도 시크릿은 필요하지 않습니다. 저장소에서 Actions
실행과 Release 게시를 허용해야 합니다. `contents: write` 권한은 Release 게시 작업에만
부여합니다.

## PR 수동 승인

PR 수동 승인 워크플로는 지정된 Pull Request를 확인하고, 조건에 맞는 경우 해당 커밋에
승인 리뷰를 제출합니다. Pull Request 이벤트로 자동 실행되지는 않습니다.

### 실행 방법

1. 저장소의 **Actions** 탭에서 **Manual PR approval** 워크플로를 엽니다.
2. **Run workflow**를 선택하고 브랜치로 `main`을 지정합니다.
3. 승인할 PR 번호를 입력하고 실행합니다.

### 승인 대상

PR 작성자는 제한하지 않습니다. 입력한 PR이 `main`을 대상으로 하고 초안이 아니며, GitHub의
리뷰 상태가 `REVIEW_REQUIRED`인 경우에만 승인 여부를 확인합니다. PR은 닫혔거나 병합된
상태여야 합니다. 열린 PR은 건너뜁니다. 동일한 커밋에 GitHub Actions 봇의 승인이 이미 있으면
중복 승인하지 않습니다. 조건을 충족하면 PR의 마지막 커밋에 승인 리뷰를 제출합니다.

### 권한과 결과 확인

승인 작업에는 `pull-requests: write` 권한이 필요합니다. 저장소 설정의 **Settings → Actions →
General → Workflow permissions**에서 **Allow GitHub Actions to create and approve pull
requests**를 활성화해야 합니다. 조건에 맞지 않아 건너뛴 경우나 GitHub API 요청이 실패한 경우는
해당 Actions 실행 로그에서 확인할 수 있습니다.
