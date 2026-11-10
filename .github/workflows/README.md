# GitHub Actions: CI와 CD

## 지속적 통합 (CI)

`ci.yml`은 `main` 브랜치에 push하거나 `main`을 대상으로 Pull Request를
생성하거나 갱신할 때 실행합니다. `ubuntu-latest`에서 다음 순서로 변경 사항을 검증합니다.

1. 저장소를 체크아웃하고 `libgtest-dev`를 설치합니다.
2. CMake로 Debug 빌드를 구성합니다.
3. 전체 타깃을 병렬 빌드합니다.
4. CTest로 전체 테스트를 실행하고 실패 시 상세 로그를 출력합니다.

CI는 `contents: read` 권한만 사용하며, 패키지나 Release를 게시하지 않습니다.
GitHub Actions의 CI 실행 또는 Pull Request의 Checks에서 결과를 확인합니다.

## 지속적 제공 (CD)

`cd.yml`은 Ubuntu 24.04 x86_64에서 Release 빌드와 전체
CTest 테스트가 성공한 경우에만 배포 패키지를 만듭니다.
CI와 CD는 독립적으로 실행하며, CD는 배포할 Release 빌드에 대해 테스트를
다시 수행합니다. 테스트가 없거나 실패하면 패키지 생성과 Release 게시를 중단합니다.

| 실행 조건 | 결과 |
| --- | --- |
| `main` 브랜치에 push | Actions 실행의 Artifacts에 패키지를 30일간 보관 |
| `vMAJOR.MINOR.PATCH` 태그 push | 패키지를 보관하고 GitHub Release에 게시 |
| Actions → CD → Run workflow | 선택한 브랜치의 패키지를 수동으로 생성 |

태그는 `main`에 포함된 커밋만 허용합니다. 예를 들어 배포할 커밋에서 다음을
실행합니다. 버전 번호는 아직 사용하지 않은 값으로 지정합니다.

```sh
git tag v1.0.0
git push origin v1.0.0
```

워크플로는 기본 `GITHUB_TOKEN`을 사용하므로 별도 시크릿이 필요하지 않습니다.
저장소에서 GitHub Actions 실행과 Release 게시를 허용해야 합니다.
Release 게시 작업에만 `contents: write` 권한을 부여합니다.
게시 전에 실패하여 draft Release가 남았다면 해당 draft를 삭제한 뒤 실패한
작업을 재실행합니다. 이미 게시한 Release는 덮어쓰지 않으며, 수정 배포에는
새 버전 태그를 사용합니다.

### 패키지 실행

`aesa-processor-<커밋 SHA>-linux-x86_64.tar.gz`와 같은 이름의 `.sha256` 파일을
함께 내려받습니다. Actions artifact는 먼저 ZIP 압축을 해제합니다.
패키지는 Ubuntu 24.04 x86_64용이며 시스템의 glibc와 libstdc++ 런타임을 사용합니다.
다른 운영체제용 바이너리는 포함하지 않습니다.

```sh
# 아래 <커밋 SHA>를 내려받은 파일명의 값으로 바꿉니다.
sha256sum --check aesa-processor-<커밋 SHA>-linux-x86_64.tar.gz.sha256
tar -xzf aesa-processor-<커밋 SHA>-linux-x86_64.tar.gz
cd aesa-processor-<커밋 SHA>-linux-x86_64
cp .env.example .env
# .env를 운용 환경에 맞게 편집한 후 실행합니다.
./adp
```

패키지에는 실행 파일, `.env.example`, 프로젝트 루트의 `README.md`,
이 문서를 복사한 `CI_CD.md`, 커밋 SHA를 기록한 `COMMIT`,
외부 라이브러리 라이선스가 포함됩니다. 실제 `.env`는 포함하지 않습니다.
운용 장비에서의 설치와 프로세스 재시작은 운영자가 수행합니다. 이전 버전으로
복구할 때는 기존 프로세스를 종료하고 해당 Release 패키지에 운용 설정을 적용한
뒤 실행합니다.
