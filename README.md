# aesa-processor
AESA 추적 레이더 데이터 처리부

## 기여자

| 이름 | 주요 역할 |
|---|---|
| [신성현](https://github.com/sin6708k) | 프로젝트 리더 |
| [박현준](https://github.com/2chasik) | 타겟 모듈 개발 |
| [이인성](https://github.com/inseong276-creator) | 통신 모듈 개발 |
| [장해찬](https://github.com/papperfield) | 오케스트레이션 모듈 개발 |
| [천영기](https://github.com/YGC20) | 빔 모듈 개발 |

## 시스템 구성

```
system    // 레이더 데이터 처리부
├─ app      // 오케스트레이션 모듈
├─ beam     // 빔 모듈
├─ comm     // 통신 모듈
└─ target   // 타겟 모듈
```

- **외부 시스템과 통신 방식**: TCP/IP
- **내부 모듈 간 통신 방식**: 이벤트 큐
